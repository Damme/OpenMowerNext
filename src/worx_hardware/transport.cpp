#include "worx_hardware/transport.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include <cerrno>
#include <cmath>
#include <cstring>

namespace open_mower_next::worx_hardware
{

// ---- spidev ---------------------------------------------------------------------

SpidevTransport::SpidevTransport(std::string device, uint32_t speed_hz, uint8_t mode)
: device_(std::move(device)), speed_hz_(speed_hz), mode_(mode)
{
}

SpidevTransport::~SpidevTransport()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool SpidevTransport::open(std::string & error)
{
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    error = "open " + device_ + ": " + std::strerror(errno) + " (check udev permissions)";
    return false;
  }
  uint8_t bits = 8;
  if (
    ::ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0 || ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
  {
    error = "configure " + device_ + ": " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

bool SpidevTransport::transfer(const Frame & tx, Frame & rx)
{
  if (fd_ < 0) {
    return false;
  }
  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<uintptr_t>(tx.data());
  tr.rx_buf = reinterpret_cast<uintptr_t>(rx.data());
  tr.len = static_cast<uint32_t>(kFrameLen);
  tr.speed_hz = speed_hz_;
  tr.bits_per_word = 8;
  tr.delay_usecs = 0;
  return ::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

// ---- fake board -------------------------------------------------------------------

FakeBoardTransport::FakeBoardTransport(Options options) : opt_(options), battery_mv_(options.battery_mv) {}

void FakeBoardTransport::setBatteryMv(int mv)
{
  std::lock_guard<std::mutex> lock(mutex_);
  battery_mv_ = mv;
}

bool FakeBoardTransport::open(std::string &) { return true; }

void FakeBoardTransport::setInCharger(bool in_charger)
{
  std::lock_guard<std::mutex> lock(mutex_);
  opt_.in_charger = in_charger;
}
int FakeBoardTransport::leftPwm() const { std::lock_guard<std::mutex> l(mutex_); return pwm_l_; }
int FakeBoardTransport::rightPwm() const { std::lock_guard<std::mutex> l(mutex_); return pwm_r_; }
int FakeBoardTransport::mowPwm() const { std::lock_guard<std::mutex> l(mutex_); return pwm_mow_; }
bool FakeBoardTransport::enabled() const { std::lock_guard<std::mutex> l(mutex_); return enabled_; }
int FakeBoardTransport::pings() const { std::lock_guard<std::mutex> l(mutex_); return pings_; }

void FakeBoardTransport::handleCommand(const std::string & msg)
{
  const auto j = nlohmann::json::parse(msg, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    return;
  }
  auto queue = [this](const std::string & s) {
    out_.push_back(kSof);
    out_.insert(out_.end(), s.begin(), s.end());
    out_.push_back(kEof);
  };
  if (j.contains("MOTORREQ_SETSPEED")) {
    const auto & s = j["MOTORREQ_SETSPEED"];
    pwm_l_ = s.value("left", 0);
    pwm_r_ = s.value("right", 0);
    pwm_mow_ = s.value("mow", 0);
    if (block_forward_) {  // motorctrl.c MOTORREQ_SETSPEED
      pwm_l_ = std::min(pwm_l_, 0);
      pwm_r_ = std::min(pwm_r_, 0);
      if (pwm_l_ < 0 || pwm_r_ < 0 || (pwm_l_ == 0 && pwm_r_ == 0)) block_forward_ = false;
    }
  } else if (j.contains("MOTORREQ_ENABLE")) {
    enabled_ = true;
    queue(R"({"motorState":"MOTORREQ_ENABLE"})");
  } else if (j.contains("MOTORREQ_DISABLE")) {
    enabled_ = false;
    queue(R"({"motorState":"MOTORREQ_DISABLE"})");
  } else if (j.contains("ping")) {
    ++pings_;
  }
  // MOTORREQ_RESETEMG: the fake board never raises an emergency itself.
}

void FakeBoardTransport::step(std::chrono::steady_clock::time_point now)
{
  if (!started_) {
    started_ = true;
    last_step_ = last_pulse_ = last_status_ = now;
    return;
  }
  const double dt = std::chrono::duration<double>(now - last_step_).count();
  last_step_ = now;
  if (opt_.in_charger && battery_mv_ < opt_.full_mv) {
    battery_mv_ = std::min<double>(opt_.full_mv, battery_mv_ + opt_.charge_mv_per_s * dt);
  }
  if (collision_ || lift_) {  // motionSensor_Timer (1 kHz)
    block_forward_ = true;
  }
  if (block_forward_) {  // a latched block also stops a wheel already driving forward
    pwm_l_ = std::min(pwm_l_, 0);
    pwm_r_ = std::min(pwm_r_, 0);
  }
  if (enabled_) {
    // Like the firmware: counters only count up, the Dir bits carry the sign.
    ticks_l_ += speed_factor_ * std::abs(pwm_l_) / opt_.pwm_per_mps * opt_.ticks_per_m * dt;
    ticks_r_ += speed_factor_ * std::abs(pwm_r_) / opt_.pwm_per_mps * opt_.ticks_per_m * dt;
  }
  auto queue = [this](const std::string & s) {
    out_.push_back(kSof);
    out_.insert(out_.end(), s.begin(), s.end());
    out_.push_back(kEof);
  };
  if (std::chrono::duration<double>(now - last_pulse_).count() >= opt_.pulse_period_s) {
    last_pulse_ = now;
    nlohmann::ordered_json p;
    p["MotorPulse"] = {
      {"Left", static_cast<long long>(ticks_l_)},
      {"Right", static_cast<long long>(ticks_r_)},
      {"Mow", enabled_ && pwm_mow_ != 0 ? 180 : 0},
      {"DirLeft", pwm_l_ < 0 ? 1 : 0},
      {"DirRight", pwm_r_ < 0 ? 1 : 0}, {"Emergancy", 0}, {"EmgReason", 0}, {"Motors", enabled_ ? 1 : 0},
      {"BlockForward", block_forward_ ? 1 : 0}, {"Bumps", bumps_},
      {"ms", static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count())}};
    queue(p.dump());
  }
  if (std::chrono::duration<double>(now - last_status_).count() >= opt_.status_period_s) {
    last_status_ = now;
    nlohmann::ordered_json b;
    switch (status_slot_) {
      case 0:
        b["Battery"] = {
          {"mV", static_cast<int>(battery_mv_)}, {"mA", opt_.in_charger ? (battery_mv_ < opt_.full_mv ? 1200 : 30) : 0},
          {"Temp", 215}, {"CellLow", 1}, {"CellHigh", 1}, {"InCharger", opt_.in_charger ? 1 : 0}};
        break;
      case 1:
        // Door/Door2/Lift/Collision read 1 when inactive.
        b["Digital"] = {{"Stuck", 1}, {"Stuck2", 1}, {"Door", 1}, {"Door2", 1}, {"Lift", lift_ ? 0 : 1},
                        {"Collision", collision_ ? 0 : 1}, {"Stop", 0}, {"Rain", 0}};
        break;
      case 2:
        b["Analog"] = {{"Rain", 3500}, {"boardTemp", 2000}};
        break;
      case 3:
        b["MotorPWM"] = {{"Left", std::abs(pwm_l_)}, {"Right", std::abs(pwm_r_)}, {"Mow", std::abs(pwm_mow_)}};
        break;
      default:
        b["MotorCurrent"] = {{"Left", std::abs(pwm_l_) / 60}, {"Right", std::abs(pwm_r_) / 60}, {"Mow", pwm_mow_ ? 300 : 0}};
        break;
    }
    status_slot_ = (status_slot_ + 1) % 5;
    queue(b.dump());
  }
}

void FakeBoardTransport::setCollision(bool active)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (active && !collision_) ++bumps_;  // firmware bump counter
  collision_ = active;
}

void FakeBoardTransport::setSpeedFactor(double factor)
{
  std::lock_guard<std::mutex> l(mutex_);
  speed_factor_ = std::max(0.0, factor);
}

void FakeBoardTransport::setLift(bool active)
{
  std::lock_guard<std::mutex> l(mutex_);
  lift_ = active;
}

bool FakeBoardTransport::blockForward() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return block_forward_;
}

bool FakeBoardTransport::transfer(const Frame & tx, Frame & rx)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> msgs;
  decoder_.feed(tx.data(), tx.size(), msgs);
  for (const auto & m : msgs) {
    handleCommand(m);
  }
  step(std::chrono::steady_clock::now());
  rx.fill(kNop);
  for (size_t i = 0; i < kFrameLen && !out_.empty(); ++i) {
    rx[i] = out_.front();
    out_.pop_front();
  }
  return true;
}

}  // namespace open_mower_next::worx_hardware

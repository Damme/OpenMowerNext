#pragma once
// SPI transports for the Worx link: the real spidev device and a fake board
// that emulates the Worx firmware (for tests and bench runs without a robot).

#include "worx_hardware/worx_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace open_mower_next::worx_hardware
{

class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool open(std::string & error) = 0;
  // Full-duplex transfer of exactly kFrameLen bytes.
  virtual bool transfer(const Frame & tx, Frame & rx) = 0;
};

class SpidevTransport : public Transport
{
public:
  // Same settings as the ROS1 worx_comms: mode 0, 8 bit, 1.5 MHz.
  explicit SpidevTransport(std::string device, uint32_t speed_hz = 1500000, uint8_t mode = 0);
  ~SpidevTransport() override;
  bool open(std::string & error) override;
  bool transfer(const Frame & tx, Frame & rx) override;

private:
  std::string device_;
  uint32_t speed_hz_;
  uint8_t mode_;
  int fd_ = -1;
};

// Emulates the Worx board: follows MOTORREQ_SETSPEED (only while enabled),
// integrates wheel ticks from PWM and reports MotorPulse / Battery / motorState.
class FakeBoardTransport : public Transport
{
public:
  struct Options
  {
    double ticks_per_m = 414.0;
    double pwm_per_mps = 1230.0;   // inverse of the host-side speed scaling
    double pulse_period_s = 0.02;  // MotorPulse rate
    double battery_period_s = 1.0;
    int battery_mv = 28000;
    bool in_charger = false;
  };

  explicit FakeBoardTransport(Options options);
  bool open(std::string & error) override;
  bool transfer(const Frame & tx, Frame & rx) override;

  // Test hooks (thread-safe).
  void setInCharger(bool in_charger);
  int leftPwm() const;
  int rightPwm() const;
  int mowPwm() const;
  bool enabled() const;
  int pings() const;

private:
  void handleCommand(const std::string & msg);
  void step(std::chrono::steady_clock::time_point now);

  Options opt_;
  mutable std::mutex mutex_;
  FrameDecoder decoder_;
  std::deque<uint8_t> out_;
  bool enabled_ = false;
  int pwm_l_ = 0, pwm_r_ = 0, pwm_mow_ = 0;
  double ticks_l_ = 0, ticks_r_ = 0;
  int pings_ = 0;
  bool started_ = false;
  std::chrono::steady_clock::time_point last_step_, last_pulse_, last_battery_;
};

}  // namespace open_mower_next::worx_hardware

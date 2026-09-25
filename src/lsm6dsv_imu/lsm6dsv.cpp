#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace open_mower_next::lsm6dsv_imu
{
namespace
{
constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL1 = 0x10;
constexpr uint8_t REG_CTRL2 = 0x11;
constexpr uint8_t REG_CTRL3 = 0x12;
constexpr uint8_t REG_CTRL6 = 0x15;
constexpr uint8_t REG_CTRL7 = 0x16;
constexpr uint8_t REG_CTRL8 = 0x17;
constexpr uint8_t REG_CTRL9 = 0x18;
constexpr uint8_t REG_STATUS = 0x1E;
constexpr uint8_t REG_OUT_TEMP_L = 0x20;
constexpr uint8_t REG_OUTX_L_G = 0x22;
constexpr uint8_t REG_OUTX_L_A = 0x28;
constexpr uint8_t EMB_FUNC_EN_A = 0x04;
constexpr uint8_t STATUS_GDA = 0x02;

constexpr double ACCEL_SCALE = 0.061e-3 * 9.80665;      // m/s^2 per LSB (+-2 g)
constexpr double GYRO_SCALE = 17.5e-3 * (M_PI / 180.0);  // rad/s per LSB (+-500 dps)
constexpr double GYRO_DEG_PER_LSB = 17.5e-3;
constexpr double TEMP_SCALE = 1.0 / 256.0;
constexpr double TEMP_OFFSET = 25.0;

inline int16_t le16(const uint8_t * b)
{
  return static_cast<int16_t>(static_cast<uint16_t>(b[1]) << 8 | b[0]);
}

std::string fmt(const char * f, double a = 0, double b = 0, double c = 0)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

class LinuxI2c : public I2cBus
{
public:
  explicit LinuxI2c(int fd) : fd_(fd) {}
  ~LinuxI2c() override { ::close(fd_); }
  int readReg8(uint8_t reg) override
  {
    uint8_t v = 0;
    if (::write(fd_, &reg, 1) != 1 || ::read(fd_, &v, 1) != 1) return -1;
    return v;
  }
  bool writeReg8(uint8_t reg, uint8_t value) override
  {
    uint8_t b[2] = {reg, value};
    return ::write(fd_, b, 2) == 2;
  }
  bool burstRead(uint8_t reg, uint8_t * buf, int len) override
  {
    return ::write(fd_, &reg, 1) == 1 && ::read(fd_, buf, len) == len;
  }

private:
  int fd_;
};
}  // namespace

std::unique_ptr<I2cBus> openLinuxI2c(const std::string & device, uint8_t address, std::string & error)
{
  const int fd = ::open(device.c_str(), O_RDWR);
  if (fd < 0) {
    error = "open " + device + ": " + std::strerror(errno);
    return nullptr;
  }
  if (::ioctl(fd, I2C_SLAVE, address) < 0) {
    error = "I2C_SLAVE: " + std::string(std::strerror(errno));
    ::close(fd);
    return nullptr;
  }
  return std::make_unique<LinuxI2c>(fd);
}

Vec3 remapSensorToBody(const Vec3 & s) { return {-s.z, s.x, -s.y}; }

Vec3 rotate(const Mat3 & R, const Vec3 & v)
{
  return {
    R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
    R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
    R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z};
}

Mat3 identity()
{
  Mat3 R{};
  for (int i = 0; i < 3; i++) R[i][i] = 1.0;
  return R;
}

Mat3 levelRotation(const Vec3 & up, double & tilt_deg, bool & ok)
{
  Mat3 R = identity();
  tilt_deg = 0.0;
  const double mag = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
  if (mag < 1.0) {
    ok = false;  // implausible (moving / no data): leave unrotated
    return R;
  }
  ok = true;
  const double ux = up.x / mag, uy = up.y / mag, uz = up.z / mag;
  tilt_deg = std::acos(std::clamp(uz, -1.0, 1.0)) * 180.0 / M_PI;
  // v = u x z, c = u . z; R = I + [v]x + [v]x^2 (1 - c) / |v|^2
  const double vx = uy, vy = -ux, vz = 0.0;
  const double s2 = vx * vx + vy * vy + vz * vz;
  const double c = uz;
  if (s2 < 1e-12) {
    if (c < 0.0) {
      R[1][1] = -1.0;
      R[2][2] = -1.0;
    }
    return R;
  }
  const double k = (1.0 - c) / s2;
  const double V[3][3] = {{0.0, -vz, vy}, {vz, 0.0, -vx}, {-vy, vx, 0.0}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double v2 = 0.0;
      for (int m = 0; m < 3; m++) v2 += V[i][m] * V[m][j];
      R[i][j] = (i == j ? 1.0 : 0.0) + V[i][j] + k * v2;
    }
  }
  return R;
}

Lsm6dsv::Lsm6dsv(std::unique_ptr<I2cBus> bus, Log log) : bus_(std::move(bus)), log_(std::move(log)) {}

bool Lsm6dsv::init()
{
  const int id = bus_->readReg8(REG_WHO_AM_I);
  if (id != kWhoAmI) {
    char b[160];
    std::snprintf(b, sizeof(b), "LSM6DSV WHO_AM_I: got 0x%02X, expected 0x%02X (address SA0 LOW=0x6A, HIGH=0x6B?)",
                  id & 0xFF, kWhoAmI);
    log_(2, b);
    return false;
  }
  bus_->writeReg8(REG_CTRL3, 0x01);  // software reset, self-clearing
  for (int i = 0; i < 200; i++) {
    if (!(bus_->readReg8(REG_CTRL3) & 0x01)) break;
    usleep(1000);
  }
  usleep(15000);
  bus_->writeReg8(REG_CTRL3, 0x44);  // BDU=1, IF_INC=1
  bus_->writeReg8(REG_CTRL6, 0x62);  // FS_G +-500 dps, LPF1_G_BW 24.2 Hz
  bus_->writeReg8(REG_CTRL7, 0x01);  // LPF1_G_EN
  bus_->writeReg8(REG_CTRL8, 0x20);  // accel LPF2 ODR/10, FS_XL +-2 g
  bus_->writeReg8(REG_CTRL9, 0x08);  // LPF2_XL_EN
  bus_->writeReg8(REG_CTRL2, 0x16);  // gyro HAODR 120 Hz
  bus_->writeReg8(REG_CTRL1, 0x16);  // accel HAODR 120 Hz
  usleep(50000);                     // gyro turn-on time
  return true;
}

void Lsm6dsv::enableSflp()
{
  bus_->writeReg8(REG_FUNC_CFG_ACCESS, 0x80);  // unlock embedded bank
  usleep(200);
  const int cur = bus_->readReg8(EMB_FUNC_EN_A);
  bus_->writeReg8(EMB_FUNC_EN_A, static_cast<uint8_t>((cur < 0 ? 0 : cur) | 0x02));  // SFLP_GAME_EN
  bus_->writeReg8(REG_FUNC_CFG_ACCESS, 0x00);
  usleep(200);
  log_(0, "SFLP gyro-bias estimation enabled");
}

bool Lsm6dsv::waitGyroReady(int polls, int poll_us)
{
  for (int i = 0; i < polls; ++i) {
    int status = bus_->readReg8(REG_STATUS);
    if (status < 0) status = 0;  // bus error: -1 & GDA would be truthy
    if (status & STATUS_GDA) return true;
    usleep(poll_us);
  }
  return false;
}

bool Lsm6dsv::calibrateGyroBias(int samples)
{
  log_(0, "Gyro bias calibration - keep the mower still...");
  double s[3] = {0, 0, 0};
  uint8_t buf[6];
  for (int n = 0; n < samples; ++n) {
    if (!waitGyroReady(5000, 200)) {
      log_(2, fmt("Gyro calibration: data-ready timeout at sample %.0f", n));
      return false;
    }
    if (!bus_->burstRead(REG_OUTX_L_G, buf, 6)) {
      log_(2, fmt("Gyro calibration: read error at sample %.0f", n));
      return false;
    }
    s[0] += le16(buf + 0);
    s[1] += le16(buf + 2);
    s[2] += le16(buf + 4);
  }
  for (int i = 0; i < 3; i++) bias_[i] = s[i] / samples;
  const auto b = gyroBiasDps();
  log_(0, fmt("Gyro bias (dps): X=%+.4f  Y=%+.4f  Z=%+.4f", b.x, b.y, b.z));
  return true;
}

Vec3 Lsm6dsv::gyroBiasDps() const
{
  return {bias_[0] * GYRO_DEG_PER_LSB, bias_[1] * GYRO_DEG_PER_LSB, bias_[2] * GYRO_DEG_PER_LSB};
}

bool Lsm6dsv::calibrateLevel(bool enabled, int samples)
{
  level_ = identity();
  if (!enabled) {
    log_(0, "Auto-level disabled");
    return true;
  }
  log_(0, "Auto-level - keep the mower still...");
  Vec3 sum;
  uint8_t buf[6];
  for (int n = 0; n < samples; ++n) {
    if (!bus_->burstRead(REG_OUTX_L_A, buf, 6)) {
      log_(2, fmt("Auto-level: accel read error at sample %.0f", n));
      return false;
    }
    const Vec3 a = remapSensorToBody({le16(buf + 0) * ACCEL_SCALE, le16(buf + 2) * ACCEL_SCALE, le16(buf + 4) * ACCEL_SCALE});
    sum.x += a.x;
    sum.y += a.y;
    sum.z += a.z;
    usleep(8000);  // ~125 Hz
  }
  const Vec3 up{sum.x / samples, sum.y / samples, sum.z / samples};
  double tilt = 0.0;
  bool ok = false;
  level_ = levelRotation(up, tilt, ok);
  if (!ok) {
    log_(1, "Auto-level: |accel| too low, leaving frame unrotated");
  } else if (tilt > 90.0) {
    log_(1, fmt("Auto-level: accel points down (tilt %.1f deg) - mounted upside down?", tilt));
  } else {
    log_(0, fmt("Auto-level: mount tilt %.2f deg corrected (Z locked to vertical)", tilt));
  }
  return true;
}

bool Lsm6dsv::read(Sample & out)
{
  if (!waitGyroReady(500, 100)) {
    // Persistent GDA=0 -> suspect POR/brownout (CTRL2 reads 0 = gyro off).
    const int c2 = bus_->readReg8(REG_CTRL2);
    char b[96];
    std::snprintf(b, sizeof(b), "IMU data-ready timeout (CTRL2=0x%02X) - dropping sample", c2 < 0 ? 0 : c2);
    log_(1, b);
    if (++timeouts_ >= 3) {
      log_(2, "IMU unresponsive - re-initializing (bias/level kept)");
      if (init()) enableSflp();
      timeouts_ = 0;
    }
    return false;
  }
  timeouts_ = 0;
  uint8_t g[6], a[6];
  if (!bus_->burstRead(REG_OUTX_L_G, g, 6) || !bus_->burstRead(REG_OUTX_L_A, a, 6)) {
    log_(1, "IMU read error - skipping sample");
    return false;
  }
  // Bias removed in raw LSB before scaling (keeps precision).
  const Vec3 gs{(le16(g + 0) - bias_[0]) * GYRO_SCALE, (le16(g + 2) - bias_[1]) * GYRO_SCALE,
                (le16(g + 4) - bias_[2]) * GYRO_SCALE};
  const Vec3 as{le16(a + 0) * ACCEL_SCALE, le16(a + 2) * ACCEL_SCALE, le16(a + 4) * ACCEL_SCALE};
  out.gyro = rotate(level_, remapSensorToBody(gs));
  out.accel = rotate(level_, remapSensorToBody(as));
  uint8_t t[2] = {0, 0};
  out.temperature = bus_->burstRead(REG_OUT_TEMP_L, t, 2) ? le16(t) * TEMP_SCALE + TEMP_OFFSET : 0.0;
  return true;
}

}  // namespace open_mower_next::lsm6dsv_imu

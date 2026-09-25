#pragma once
// ST LSM6DSV IMU driver (datasheet DS13476 Rev 5), ported from the ROS1
// lsm6dsv_imu node of the Worx robot. ROS-free; the I2C bus is abstract so the
// driver can be tested against a fake chip.
//
// Settings: gyro 120 Hz HAODR +-500 dps LPF1 24 Hz, accel 120 Hz HAODR +-2 g
// LPF2 12 Hz, SFLP on-chip gyro-bias estimation. Static gyro-bias calibration
// and an auto-level rotation (gravity -> +Z) at startup.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace open_mower_next::lsm6dsv_imu
{

class I2cBus
{
public:
  virtual ~I2cBus() = default;
  virtual int readReg8(uint8_t reg) = 0;                         // <0 on error
  virtual bool writeReg8(uint8_t reg, uint8_t value) = 0;
  virtual bool burstRead(uint8_t reg, uint8_t * buf, int len) = 0;
};

// /dev/i2c-N via i2c-dev (what wiringPiI2C used underneath).
std::unique_ptr<I2cBus> openLinuxI2c(const std::string & device, uint8_t address, std::string & error);

using Mat3 = std::array<std::array<double, 3>, 3>;
struct Vec3
{
  double x = 0, y = 0, z = 0;
};

// Mounting remap sensor -> body (REP-103), the Worx board orientation:
// body x = -z_s, y = +x_s, z = -y_s.
Vec3 remapSensorToBody(const Vec3 & s);
Vec3 rotate(const Mat3 & R, const Vec3 & v);
Mat3 identity();
// Minimal rotation that maps the measured "up" (body frame accel) onto +Z.
// Returns identity if |up| < 1 m/s^2; 180 deg flip about X if upside down.
Mat3 levelRotation(const Vec3 & up, double & tilt_deg, bool & ok);

struct Sample
{
  Vec3 gyro;   // rad/s, body frame, bias removed, levelled
  Vec3 accel;  // m/s^2, body frame, levelled
  double temperature = 0.0;  // degC
};

class Lsm6dsv
{
public:
  using Log = std::function<void(int level /*0 info,1 warn,2 error*/, const std::string &)>;
  static constexpr uint8_t kWhoAmI = 0x70;

  Lsm6dsv(std::unique_ptr<I2cBus> bus, Log log);

  bool init();                 // WHO_AM_I, reset, configure
  void enableSflp();
  bool calibrateGyroBias(int samples = 256);
  bool calibrateLevel(bool enabled, int samples = 128);

  // Wait for gyro data-ready (up to ~50 ms) and read one sample. Returns false
  // on timeout/read error; after 3 consecutive timeouts re-initializes the chip
  // (bias and level are kept).
  bool read(Sample & out);

  Vec3 gyroBiasDps() const;

private:
  bool waitGyroReady(int polls, int poll_us);

  std::unique_ptr<I2cBus> bus_;
  Log log_;
  double bias_[3] = {0, 0, 0};  // raw LSB
  Mat3 level_ = identity();
  int timeouts_ = 0;
};

}  // namespace open_mower_next::lsm6dsv_imu

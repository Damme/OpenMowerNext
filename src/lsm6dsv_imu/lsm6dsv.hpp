#pragma once
// ST LSM6DSV IMU driver (datasheet DS13476 Rev 5), ported from the ROS1
// lsm6dsv_imu node of the Worx robot. ROS-free; the I2C bus is abstract so the
// driver can be tested against a fake chip.
//
// Settings: gyro 120 Hz +-500 dps LPF1 24 Hz, accel 120 Hz +-2 g LPF2 12 Hz
// (high-performance mode, as ST's SFLP example). Static gyro-bias calibration
// (standing still, checked: the gyro must not move) at startup and on request;
// the level rotation (gravity -> +Z) from a fixed mount calibration (setLevel)
// or measured at startup (calibrateLevel: whatever the robot stands on is
// taken as level - the dock slope gave 12.6-19.1 deg for the same mount).
//
// FIFO mode (enableFifo): the chip batches every gyro sample (60 Hz) and accel
// sample (15 Hz) and, from its SFLP sensor fusion, its running gyro-bias
// estimate (15 Hz). read() then returns the MEAN of everything since the last
// call - no single-sample aliasing (blade vibration near the publish rate) - with
// the bias removed. SFLP's bias is NOT applied by the chip to the output registers
// (ST). Its absolute value sat ~0.003 dps off the stationary truth, so only its
// change is used: bias = startup calibration + (SFLP - SFLP reference), the
// reference taken once SFLP agreed with the startup calibration (convergence,
// axes, sign) and settled; within a band, else the startup bias.
// SFLP tracking is off by default: on the robot it wandered only while mowing
// (2026-10-01..20: Z up to 0.30 dps off, held for half an hour; docked it stays
// within 0.006 dps), and every error turns straight into heading drift.

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

struct FifoOptions
{
  bool sflp_bias = false;       // track SFLP's gyro-bias estimate (see above)
  double bias_agree_dps = 0.05; // SFLP must once come this close to the startup bias (all axes)
  double bias_band_dps = 0.5;   // max change of SFLP's bias from its reference (all axes)
  double bias_tau = 30.0;       // s: low-pass on SFLP's bias (4.375 mdps steps -> sub-step resolution)
};

struct Sample
{
  Vec3 gyro;   // rad/s, body frame, bias removed, levelled
  Vec3 accel;  // m/s^2, body frame, levelled
  double temperature = 0.0;  // degC
  int gyro_samples = 1;      // chip samples averaged into this one (FIFO mode)
};

class Lsm6dsv
{
public:
  using Log = std::function<void(int level /*0 info,1 warn,2 error*/, const std::string &)>;
  static constexpr uint8_t kWhoAmI = 0x70;

  Lsm6dsv(std::unique_ptr<I2cBus> bus, Log log);

  bool init();                 // WHO_AM_I, reset, configure
  // Mean of the gyro output registers (256 samples ~2.1 s). Refused, keeping the
  // old bias, when the robot moves: any axis' standard deviation above
  // max_std_dps, or (after a first calibration) any axis' mean more than
  // max_change_dps from the current bias - a slow steady turn. 0 = no check.
  // In FIFO mode the FIFO is emptied afterwards. Clears needsCalibration().
  bool calibrateGyroBias(int samples = 256, double max_std_dps = 0.0, double max_change_dps = 0.0);
  bool calibrateLevel(bool enabled, int samples = 128);
  // Mean accel (body frame, NOT levelled) - "up" for setLevel - while standing still.
  bool measureUp(Vec3 & up, int samples = 128);
  // Fixed level rotation from a measured mount "up" (body frame, any length).
  bool setLevel(const Vec3 & up);
  // After the calibrations: FIFO stream mode (+ SFLP gyro bias). Without it
  // read() takes single samples from the output registers.
  bool enableFifo(const FifoOptions & options);

  // Register mode: wait for gyro data-ready (up to ~50 ms), one sample.
  // FIFO mode: mean of the FIFO since the last call; false if it holds no gyro
  // sample. Either way, after 3 consecutive misses the chip is re-initialized
  // (bias and level are kept).
  bool read(Sample & out);

  Vec3 gyroBiasDps() const;      // startup calibration
  Vec3 activeBiasDps() const;    // what read() subtracts
  bool sflpBiasInUse() const { return sflp_in_use_; }
  bool hasSflpBias() const { return sflp_seen_; }
  Vec3 sflpBiasDps() const;      // SFLP's latest estimate (sensor frame)
  // The chip stopped delivering and was re-initialized: the mower should stop and
  // calibrate the gyro again before it trusts the heading.
  bool needsCalibration() const { return needs_calibration_; }
  int resets() const { return resets_; }
  const std::string & lastError() const { return last_error_; }

private:
  bool waitGyroReady(int polls, int poll_us);
  bool readRegisters(Sample & out);
  bool readFifo(Sample & out);
  bool configureFifo();
  void onSflpBias(const int16_t raw[3]);
  void missed();

  std::unique_ptr<I2cBus> bus_;
  Log log_;
  double bias_[3] = {0, 0, 0};  // raw LSB, startup calibration
  double active_bias_[3] = {0, 0, 0};  // raw LSB, subtracted by read()
  double sflp_bias_[3] = {0, 0, 0};    // raw gyro LSB (converted from SFLP's 4.375 mdps/LSB), low-passed
  double sflp_ref_[3] = {0, 0, 0};     // sflp_bias_ when it had settled after convergence
  bool sflp_seen_ = false, sflp_agreed_ = false, sflp_in_use_ = false;
  bool sflp_ref_set_ = false, sflp_band_left_ = false;
  double sflp_sum_[3] = {0, 0, 0};     // for the reference: mean over 2 tau after convergence
  int sflp_sum_n_ = 0;
  bool fifo_ = false;
  bool burst_ = true;  // several FIFO words per I2C transaction
  FifoOptions fifo_opts_;
  Mat3 level_ = identity();
  Vec3 last_accel_;
  int timeouts_ = 0;
  bool calibrated_ = false, needs_calibration_ = false;
  int resets_ = 0;
  std::string last_error_;
};

}  // namespace open_mower_next::lsm6dsv_imu

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
constexpr uint8_t REG_FIFO_CTRL3 = 0x09;       // BDR_GY[7:4], BDR_XL[3:0]
constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;       // FIFO_MODE[2:0]
constexpr uint8_t REG_FIFO_STATUS1 = 0x1B;     // DIFF_FIFO[7:0]
constexpr uint8_t REG_FIFO_STATUS2 = 0x1C;     // bit0 DIFF_FIFO[8], bit6 FIFO_OVR_IA
constexpr uint8_t REG_FIFO_DATA_OUT_TAG = 0x78;  // tag + 6 data bytes
// Embedded-functions bank (FUNC_CFG_ACCESS = 0x80):
constexpr uint8_t EMB_FUNC_FIFO_EN_A = 0x44;   // bit5 SFLP_GBIAS_FIFO_EN
constexpr uint8_t EMB_SFLP_ODR = 0x5E;         // SFLP_GAME_ODR[5:3]
constexpr uint8_t TAG_GYRO = 0x01, TAG_ACCEL = 0x02, TAG_SFLP_GBIAS = 0x16;
constexpr double SFLP_GBIAS_TO_GYRO_LSB = 4.375 / 17.5;  // 4.375 mdps/LSB -> +-500 dps LSB
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
  bus_->writeReg8(REG_CTRL2, 0x06);  // gyro 120 Hz, high-performance mode (FIFO batches 60 Hz)
  bus_->writeReg8(REG_CTRL1, 0x06);  // accel 120 Hz, high-performance mode (FIFO batches 15 Hz)
  usleep(50000);                     // gyro turn-on time
  return true;
}

bool Lsm6dsv::enableFifo(const FifoOptions & options)
{
  fifo_opts_ = options;
  if (!configureFifo()) return false;
  fifo_ = true;
  log_(0, options.sflp_bias ? "FIFO on: gyro 60 Hz, accel 15 Hz averaged per read; SFLP gyro bias 15 Hz"
                            : "FIFO on: gyro 60 Hz, accel 15 Hz averaged per read");
  return true;
}

bool Lsm6dsv::configureFifo()
{
  bool ok = bus_->writeReg8(REG_FIFO_CTRL4, 0x00);  // bypass: empties the FIFO
  // Gyro 60 Hz (LPF1 24 Hz), accel 15 Hz (not fused; spare): the Worx I2C bus runs at
  // 50 kHz, and 120 + 60 Hz one word per transaction left the node at ~7 Hz.
  ok = ok && bus_->writeReg8(REG_FIFO_CTRL3, 0x53);
  if (ok && fifo_opts_.sflp_bias) {
    // As ST's lsm6dsv_sensor_fusion example: SFLP output to FIFO, ODR, enable.
    ok = bus_->writeReg8(REG_FUNC_CFG_ACCESS, 0x80);  // embedded-functions bank
    usleep(200);
    const int odr = bus_->readReg8(EMB_SFLP_ODR);
    ok = ok && odr >= 0;
    ok = ok && bus_->writeReg8(EMB_FUNC_FIFO_EN_A, 0x20);  // SFLP_GBIAS_FIFO_EN
    ok = ok && bus_->writeReg8(EMB_SFLP_ODR, static_cast<uint8_t>(odr & ~0x38));  // 15 Hz
    const int en = bus_->readReg8(EMB_FUNC_EN_A);
    ok = ok && en >= 0 && bus_->writeReg8(EMB_FUNC_EN_A, static_cast<uint8_t>(en | 0x02));  // SFLP_GAME_EN
    bus_->writeReg8(REG_FUNC_CFG_ACCESS, 0x00);
    usleep(200);
  }
  ok = ok && bus_->writeReg8(REG_FIFO_CTRL4, 0x06);  // continuous (stream) mode
  if (!ok) log_(2, "FIFO/SFLP configuration failed (I2C write)");
  return ok;
}

void Lsm6dsv::onSflpBias(const int16_t raw[3])
{
  // SFLP's absolute bias sat ~0.003 dps below the stationary truth on the robot,
  // so only its CHANGE is used: bias = startup calibration + (SFLP - SFLP reference).
  // Convergence is judged on the raw estimate. The reference is the plain mean of
  // the outputs over the following 2 tau (15 Hz, 4.375 mdps steps); after that the
  // estimate is low-passed (tau), starting from the reference.
  double v[3];
  bool agree = true;
  for (int i = 0; i < 3; i++) {
    v[i] = raw[i] * SFLP_GBIAS_TO_GYRO_LSB;
    agree = agree && std::abs(v[i] - bias_[i]) * GYRO_DEG_PER_LSB <= fifo_opts_.bias_agree_dps;
  }
  sflp_seen_ = true;
  if (sflp_ref_set_) {
    const double a = fifo_opts_.bias_tau > 0 ? std::min(1.0, 1.0 / (15.0 * fifo_opts_.bias_tau)) : 1.0;
    for (int i = 0; i < 3; i++) sflp_bias_[i] += a * (v[i] - sflp_bias_[i]);
  } else {
    for (int i = 0; i < 3; i++) sflp_bias_[i] = v[i];
    if (!sflp_agreed_) {
      if (!agree) return;
      sflp_agreed_ = true;
      const auto b = sflpBiasDps();
      log_(0, fmt("SFLP gyro bias converged to the startup calibration (dps): %+.4f %+.4f %+.4f", b.x, b.y, b.z));
    }
    for (int i = 0; i < 3; i++) sflp_sum_[i] += v[i];
    if (++sflp_sum_n_ < std::max(1, static_cast<int>(2.0 * 15.0 * fifo_opts_.bias_tau))) return;
    for (int i = 0; i < 3; i++) sflp_bias_[i] = sflp_ref_[i] = sflp_sum_[i] / sflp_sum_n_;
    sflp_ref_set_ = true;
    const auto b = sflpBiasDps();
    log_(0, fmt("SFLP reference taken, tracking bias changes from now (dps): %+.4f %+.4f %+.4f", b.x, b.y, b.z));
  }
  bool in_band = true;
  for (int i = 0; i < 3; i++) {
    in_band = in_band && std::abs(sflp_bias_[i] - sflp_ref_[i]) * GYRO_DEG_PER_LSB <= fifo_opts_.bias_band_dps;
  }
  const bool use = fifo_opts_.sflp_bias && in_band;
  if (use != sflp_in_use_ && (sflp_in_use_ || sflp_band_left_)) {
    const auto b = sflpBiasDps();
    log_(use ? 0 : 1, fmt(use ? "SFLP gyro bias change back within the band (dps): %+.4f %+.4f %+.4f"
                              : "SFLP gyro bias changed by more than the band, using the startup bias "
                                "(dps): %+.4f %+.4f %+.4f",
                          b.x, b.y, b.z));
    sflp_band_left_ = !use;
  }
  sflp_in_use_ = use;
  for (int i = 0; i < 3; i++) active_bias_[i] = use ? bias_[i] + sflp_bias_[i] - sflp_ref_[i] : bias_[i];
}

Vec3 Lsm6dsv::sflpBiasDps() const
{
  return {sflp_bias_[0] * GYRO_DEG_PER_LSB, sflp_bias_[1] * GYRO_DEG_PER_LSB, sflp_bias_[2] * GYRO_DEG_PER_LSB};
}

Vec3 Lsm6dsv::activeBiasDps() const
{
  return {active_bias_[0] * GYRO_DEG_PER_LSB, active_bias_[1] * GYRO_DEG_PER_LSB, active_bias_[2] * GYRO_DEG_PER_LSB};
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
  for (int i = 0; i < 3; i++) active_bias_[i] = bias_[i] = s[i] / samples;
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
  return fifo_ ? readFifo(out) : readRegisters(out);
}

void Lsm6dsv::missed()
{
  if (++timeouts_ >= 3) {
    log_(2, "IMU unresponsive - re-initializing (bias/level kept)");
    if (init() && fifo_) configureFifo();
    timeouts_ = 0;
  }
}

bool Lsm6dsv::readFifo(Sample & out)
{
  const int s1 = bus_->readReg8(REG_FIFO_STATUS1);
  const int s2 = bus_->readReg8(REG_FIFO_STATUS2);
  if (s1 < 0 || s2 < 0) {
    log_(1, "IMU FIFO status read error - skipping sample");
    missed();
    return false;
  }
  if (s2 & 0x40) log_(1, "IMU FIFO overrun - samples lost");
  const int level = std::min(s1 | (s2 & 0x01) << 8, 128);
  long g[3] = {0, 0, 0}, a[3] = {0, 0, 0};
  int gn = 0, an = 0;
  // Several words per transaction: the address rolls back from FIFO_DATA_OUT_Z_H
  // to FIFO_DATA_OUT_TAG. If a burst yields an unknown tag, that isn't so on this
  // chip: fall back to one word per transaction (only the first word was popped then;
  // the rest stays in the FIFO for the next read).
  constexpr int kBurstWords = 16;
  uint8_t w[7 * kBurstWords];
  for (int done = 0; done < level;) {
    const int k = burst_ ? std::min(kBurstWords, level - done) : 1;
    if (!bus_->burstRead(REG_FIFO_DATA_OUT_TAG, w, 7 * k)) {
      log_(1, "IMU FIFO read error");
      break;
    }
    done += k;
    bool bad = false;
    for (int j = 0; j < k; j++) {
      const uint8_t * e = w + 7 * j;
      const int16_t v[3] = {le16(e + 1), le16(e + 3), le16(e + 5)};
      switch (e[0] >> 3) {
        case TAG_GYRO:
          for (int m = 0; m < 3; m++) g[m] += v[m];
          gn++;
          break;
        case TAG_ACCEL:
          for (int m = 0; m < 3; m++) a[m] += v[m];
          an++;
          break;
        case TAG_SFLP_GBIAS:
          onSflpBias(v);
          break;
        default:
          bad = bad || j > 0;  // only words after the first depend on the roll-back
          break;
      }
    }
    if (bad && burst_) {
      burst_ = false;
      log_(1, "IMU FIFO: multi-word burst not supported here - reading one word per transaction");
    }
  }
  if (gn == 0) {
    // 60 Hz gyro, 50 Hz reads: an empty read is scheduling jitter; three in a row re-init.
    missed();
    return false;
  }
  timeouts_ = 0;
  // Bias removed from the mean in raw LSB before scaling (keeps precision).
  const Vec3 gs{(static_cast<double>(g[0]) / gn - active_bias_[0]) * GYRO_SCALE,
                (static_cast<double>(g[1]) / gn - active_bias_[1]) * GYRO_SCALE,
                (static_cast<double>(g[2]) / gn - active_bias_[2]) * GYRO_SCALE};
  out.gyro = rotate(level_, remapSensorToBody(gs));
  if (an > 0) {
    const Vec3 as{static_cast<double>(a[0]) / an * ACCEL_SCALE, static_cast<double>(a[1]) / an * ACCEL_SCALE,
                  static_cast<double>(a[2]) / an * ACCEL_SCALE};
    last_accel_ = rotate(level_, remapSensorToBody(as));
  }
  out.accel = last_accel_;
  out.gyro_samples = gn;
  uint8_t t[2] = {0, 0};
  out.temperature = bus_->burstRead(REG_OUT_TEMP_L, t, 2) ? le16(t) * TEMP_SCALE + TEMP_OFFSET : 0.0;
  return true;
}

bool Lsm6dsv::readRegisters(Sample & out)
{
  if (!waitGyroReady(500, 100)) {
    // Persistent GDA=0 -> suspect POR/brownout (CTRL2 reads 0 = gyro off).
    const int c2 = bus_->readReg8(REG_CTRL2);
    char b[96];
    std::snprintf(b, sizeof(b), "IMU data-ready timeout (CTRL2=0x%02X) - dropping sample", c2 < 0 ? 0 : c2);
    log_(1, b);
    missed();
    return false;
  }
  timeouts_ = 0;
  uint8_t g[6], a[6];
  if (!bus_->burstRead(REG_OUTX_L_G, g, 6) || !bus_->burstRead(REG_OUTX_L_A, a, 6)) {
    log_(1, "IMU read error - skipping sample");
    return false;
  }
  // Bias removed in raw LSB before scaling (keeps precision).
  const Vec3 gs{(le16(g + 0) - active_bias_[0]) * GYRO_SCALE, (le16(g + 2) - active_bias_[1]) * GYRO_SCALE,
                (le16(g + 4) - active_bias_[2]) * GYRO_SCALE};
  const Vec3 as{le16(a + 0) * ACCEL_SCALE, le16(a + 2) * ACCEL_SCALE, le16(a + 4) * ACCEL_SCALE};
  out.gyro = rotate(level_, remapSensorToBody(gs));
  out.accel = rotate(level_, remapSensorToBody(as));
  uint8_t t[2] = {0, 0};
  out.temperature = bus_->burstRead(REG_OUT_TEMP_L, t, 2) ? le16(t) * TEMP_SCALE + TEMP_OFFSET : 0.0;
  return true;
}

}  // namespace open_mower_next::lsm6dsv_imu

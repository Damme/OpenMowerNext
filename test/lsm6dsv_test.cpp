// LSM6DSV driver against a fake chip: bias calibration (and its motion check),
// axis remap, levelling, FIFO/SFLP, re-init after the chip stopped delivering.
#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

using namespace open_mower_next::lsm6dsv_imu;

namespace
{
constexpr double ACCEL_SCALE = 0.061e-3 * 9.80665;
constexpr double GYRO_SCALE = 17.5e-3 * (M_PI / 180.0);

struct FakeChip : I2cBus
{
  std::map<uint8_t, uint8_t> regs{{0x0F, 0x70}, {0x1E, 0x02}};
  int16_t gyro[3] = {10, -20, 5};   // raw LSB, sensor frame (the bias)
  int16_t accel[3] = {0, 0, 0};
  std::deque<std::array<uint8_t, 7>> fifo;  // tag byte + 3 x int16
  bool no_rollover = false;
  int16_t wobble = 0;  // gyro register reads alternate +-wobble (vibration / moving)
  int gyro_reads = 0;
  bool gone = false;   // no answer on the bus (connection open)
  void push(uint8_t tag, int16_t x, int16_t y, int16_t z)
  {
    fifo.push_back({static_cast<uint8_t>(tag << 3), static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8 & 0xFF),
                    static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>(y >> 8 & 0xFF),
                    static_cast<uint8_t>(z & 0xFF), static_cast<uint8_t>(z >> 8 & 0xFF)});
  }
  int readReg8(uint8_t reg) override
  {
    if (gone) return -1;
    if (reg == 0x12) return regs[reg] & ~0x01;  // reset bit self-clears
    if (reg == 0x1B) return static_cast<int>(fifo.size() & 0xFF);
    if (reg == 0x1C) return static_cast<int>(fifo.size() >> 8 & 0x01);
    return regs.count(reg) ? regs[reg] : 0;
  }
  bool writeReg8(uint8_t reg, uint8_t v) override
  {
    if (gone) return false;
    regs[reg] = v;
    if (reg == 0x0A && v == 0x00) fifo.clear();  // bypass mode empties the FIFO
    return true;
  }
  bool burstRead(uint8_t reg, uint8_t * buf, int len) override
  {
    if (gone) return false;
    if (reg == 0x78 && len % 7 == 0) {
      // Burst: the address rolls back to the tag register after each word,
      // unless no_rollover (then the bytes after the first word read as 0).
      std::memset(buf, 0, len);
      for (int i = 0; i < len / 7 && !fifo.empty(); i++) {
        if (i > 0 && no_rollover) break;
        std::memcpy(buf + 7 * i, fifo.front().data(), 7);
        fifo.pop_front();
      }
      return true;
    }
    int16_t moved[3];
    if (reg == 0x22 && wobble) {
      const int16_t w = gyro_reads++ % 2 ? wobble : static_cast<int16_t>(-wobble);
      for (int i = 0; i < 3; i++) moved[i] = static_cast<int16_t>(gyro[i] + w);
    }
    const int16_t * src = reg == 0x22 ? (wobble ? moved : gyro) : reg == 0x28 ? accel : nullptr;
    if (!src) {
      std::memset(buf, 0, len);
      return true;
    }
    for (int i = 0; i < len / 2; i++) {
      buf[2 * i] = static_cast<uint8_t>(src[i] & 0xFF);
      buf[2 * i + 1] = static_cast<uint8_t>((src[i] >> 8) & 0xFF);
    }
    return true;
  }
};

// Sensor-frame raw accel for a body-frame "up" vector (inverse of the remap
// body = (-z_s, x_s, -y_s) -> s = (y_b, -z_b, -x_b)).
void setBodyAccel(FakeChip & c, double bx, double by, double bz)
{
  c.accel[0] = static_cast<int16_t>(std::lround(by / ACCEL_SCALE));
  c.accel[1] = static_cast<int16_t>(std::lround(-bz / ACCEL_SCALE));
  c.accel[2] = static_cast<int16_t>(std::lround(-bx / ACCEL_SCALE));
}
}  // namespace

TEST(Lsm6dsv, RemapMatchesWorxMounting)
{
  const Vec3 b = remapSensorToBody({1, 2, 3});
  EXPECT_DOUBLE_EQ(b.x, -3);
  EXPECT_DOUBLE_EQ(b.y, 1);
  EXPECT_DOUBLE_EQ(b.z, -2);
}

TEST(Lsm6dsv, LevelRotationMapsUpToZ)
{
  double tilt;
  bool ok;
  const Vec3 up{0.5, -0.3, 9.7};
  const Mat3 R = levelRotation(up, tilt, ok);
  ASSERT_TRUE(ok);
  const Vec3 r = rotate(R, up);
  const double mag = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
  EXPECT_NEAR(r.x, 0.0, 1e-9);
  EXPECT_NEAR(r.y, 0.0, 1e-9);
  EXPECT_NEAR(r.z, mag, 1e-9);
  EXPECT_NEAR(tilt, std::acos(up.z / mag) * 180 / M_PI, 1e-9);

  levelRotation({0, 0, 0.1}, tilt, ok);
  EXPECT_FALSE(ok);
  const Mat3 F = levelRotation({0, 0, -9.8}, tilt, ok);
  EXPECT_NEAR(rotate(F, {0, 0, -9.8}).z, 9.8, 1e-9);
}

TEST(Lsm6dsv, BiasAndLevelCalibration)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.4, -0.2, 9.79);  // mount tilted ~2.6 deg
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  EXPECT_EQ(c->regs[0x11], 0x06);  // gyro 120 Hz high-performance
  EXPECT_EQ(c->regs[0x15], 0x62);  // +-500 dps
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.calibrateLevel(true, 8));

  Sample s;
  ASSERT_TRUE(imu.read(s));
  EXPECT_NEAR(s.gyro.x, 0.0, 1e-9);  // bias removed
  EXPECT_NEAR(s.gyro.y, 0.0, 1e-9);
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
  EXPECT_NEAR(s.accel.x, 0.0, 0.01);  // levelled
  EXPECT_NEAR(s.accel.y, 0.0, 0.01);
  EXPECT_NEAR(s.accel.z, 9.8, 0.02);

  // Yaw left at 0.5 rad/s = rotation about body +Z = sensor -Y.
  c->gyro[1] = static_cast<int16_t>(-20 - std::lround(0.5 / GYRO_SCALE));
  ASSERT_TRUE(imu.read(s));
  EXPECT_NEAR(s.gyro.z, 0.5, 0.01);
}

TEST(Lsm6dsv, WrongWhoAmIFails)
{
  auto chip = std::make_unique<FakeChip>();
  chip->regs[0x0F] = 0x6C;
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  EXPECT_FALSE(imu.init());
}

TEST(Lsm6dsv, FifoAveragesAndTracksSflpBias)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));  // bias = (10, -20, 5) LSB
  ASSERT_TRUE(imu.calibrateLevel(true, 8));
  FifoOptions opt;
  opt.sflp_bias = true;
  opt.bias_tau = 0.0;  // follow every SFLP value (smoothing: own test)
  ASSERT_TRUE(imu.enableFifo(opt));
  EXPECT_EQ(c->regs[0x0A], 0x06);  // stream mode
  EXPECT_EQ(c->regs[0x09], 0x53);  // gyro 60 Hz, accel 15 Hz
  EXPECT_EQ(c->regs[0x44], 0x20);  // SFLP gyro bias into the FIFO

  Sample s;
  EXPECT_FALSE(imu.read(s));  // empty FIFO

  // Yaw rate = sensor -Y. Three samples: mean 0.5 rad/s although they differ.
  const int16_t r = static_cast<int16_t>(std::lround(0.5 / GYRO_SCALE));
  c->push(0x01, 10, static_cast<int16_t>(-20 - r + 40), 5);
  c->push(0x02, c->accel[0], c->accel[1], c->accel[2]);
  c->push(0x01, 10, static_cast<int16_t>(-20 - r - 40), 5);
  c->push(0x01, 10, static_cast<int16_t>(-20 - r), 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_EQ(s.gyro_samples, 3);
  EXPECT_NEAR(s.gyro.z, 0.5, 0.005);
  EXPECT_NEAR(s.accel.z, 9.8, 0.02);
  EXPECT_FALSE(imu.sflpBiasInUse());

  // SFLP bias (4.375 mdps/LSB = gyro LSB / 4) far from the startup bias: not converged, not used.
  c->push(0x16, 0, 0, 0);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.hasSflpBias());
  EXPECT_FALSE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);

  // Converged (agrees with the startup bias; tau 0: reference taken at once), 1 LSB off
  // the startup value: its offset is NOT used, only later changes are.
  c->push(0x16, 40, -84, 20);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
  // Drifts a little (Z bias sensor -Y: -20 -> -24 LSB, SFLP -84 -> -100): follows the change.
  c->push(0x16, 40, -100, 20);
  c->push(0x01, 10, -24, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
  // Runs away (changes > 0.5 dps from its reference): back to the startup bias.
  c->push(0x16, 40, -2000, 20);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_FALSE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
}

TEST(Lsm6dsv, FifoFallsBackToSingleWordReads)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.calibrateLevel(true, 8));
  ASSERT_TRUE(imu.enableFifo(FifoOptions{}));
  c->no_rollover = true;
  const int16_t r = static_cast<int16_t>(std::lround(0.5 / GYRO_SCALE));
  Sample s;
  // First burst: only word 1 is real, the rest reads as tag 0 -> switch to single words.
  for (int i = 0; i < 4; i++) c->push(0x01, 10, static_cast<int16_t>(-20 - r), 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_NEAR(s.gyro.z, 0.5, 0.005);  // the one real word
  for (int i = 0; i < 4; i++) c->push(0x01, 10, static_cast<int16_t>(-20 - r), 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_EQ(s.gyro_samples, 7);  // 3 left over from the first read (only word 1 popped) + 4
  EXPECT_NEAR(s.gyro.z, 0.5, 0.005);
}

TEST(Lsm6dsv, SflpBiasSmoothedBetweenSteps)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));  // Z bias sensor -Y = -20 LSB = -80 SFLP LSB
  ASSERT_TRUE(imu.calibrateLevel(true, 8));
  FifoOptions opt;
  opt.sflp_bias = true;  // tau 30 s
  ASSERT_TRUE(imu.enableFifo(opt));
  Sample s;
  // SFLP flips between two steps around the true bias (-80 +- 2): the used bias converges to the middle.
  for (int i = 0; i < 15 * 120; i++) {
    c->push(0x16, 40, static_cast<int16_t>(i % 2 ? -78 : -82), 20);
    c->push(0x01, 10, -20, 5);
    ASSERT_TRUE(imu.read(s));
  }
  EXPECT_TRUE(imu.sflpBiasInUse());
  EXPECT_NEAR(imu.activeBiasDps().y, -20 * 17.5e-3, 0.0005);
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-4);
}

TEST(Lsm6dsv, SflpOffByDefault)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  // Left on by an earlier run (the embedded bank survives the software reset).
  c->regs[0x44] = 0x20;
  c->regs[0x04] = 0x02;
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.enableFifo(FifoOptions{}));
  EXPECT_EQ(c->regs[0x44], 0x00);  // no SFLP output in the FIFO
  EXPECT_EQ(c->regs[0x04], 0x00);  // SFLP off
  // Even an SFLP word that would agree is never used.
  c->push(0x16, 40, -100, 20);
  c->push(0x01, 10, -20, 5);
  Sample s;
  ASSERT_TRUE(imu.read(s));
  EXPECT_FALSE(imu.sflpBiasInUse());
}

TEST(Lsm6dsv, CalibrationRefusedWhileMoving)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());

  c->wobble = 20;  // 0.35 dps std: vibrating / rocking
  EXPECT_FALSE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_NE(imu.lastError().find("moving"), std::string::npos);
  c->wobble = 0;
  ASSERT_TRUE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_NEAR(imu.gyroBiasDps().y, -20 * 17.5e-3, 1e-9);

  // A slow steady turn: no spread, but the mean moved 0.875 dps - refused, old bias kept.
  c->gyro[1] = -70;
  EXPECT_FALSE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_NEAR(imu.gyroBiasDps().y, -20 * 17.5e-3, 1e-9);
  // A real drift of 0.175 dps standing still: accepted.
  c->gyro[1] = -30;
  ASSERT_TRUE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_NEAR(imu.gyroBiasDps().y, -30 * 17.5e-3, 1e-9);
  EXPECT_TRUE(imu.lastError().empty());
}

TEST(Lsm6dsv, ResetNeedsCalibrationAndLogsRegisters)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  std::vector<std::string> errors;
  Lsm6dsv imu(std::move(chip), [&errors](int level, const std::string & m) {
    if (level == 2) errors.push_back(m);
  });
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.enableFifo(FifoOptions{}));
  EXPECT_FALSE(imu.needsCalibration());

  // Power loss: registers back at 0, nothing in the FIFO. Three empty reads -> re-init.
  c->regs[0x10] = c->regs[0x11] = c->regs[0x0A] = 0;
  Sample s;
  for (int i = 0; i < 3; i++) EXPECT_FALSE(imu.read(s));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("CTRL1 0 CTRL2 0 FIFO_CTRL4 0"), std::string::npos) << errors[0];
  EXPECT_TRUE(imu.needsCalibration());
  EXPECT_EQ(imu.resets(), 1);
  EXPECT_EQ(c->regs[0x11], 0x06);  // configured again
  EXPECT_EQ(c->regs[0x0A], 0x06);

  // Still flagged while it delivers again; a calibration clears it and empties the FIFO.
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.needsCalibration());
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_FALSE(imu.needsCalibration());
  EXPECT_TRUE(c->fifo.empty());
}

TEST(Lsm6dsv, FixedMountLevel)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  // Measured on flat ground: mount tilted 13 deg about body Y.
  const double t = 13.0 * M_PI / 180.0;
  const Vec3 mount{-9.8 * std::sin(t), 0.0, 9.8 * std::cos(t)};
  // Now standing on the dock, which slopes another 5 deg: the fixed level leaves the slope.
  const double d = 18.0 * M_PI / 180.0;
  setBodyAccel(*c, -9.8 * std::sin(d), 0.0, 9.8 * std::cos(d));
  Lsm6dsv imu(std::move(chip), [](int, const std::string &) {});
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.setLevel(mount));
  Vec3 up;
  ASSERT_TRUE(imu.measureUp(up, 4));  // raw, not levelled
  EXPECT_NEAR(up.x, -9.8 * std::sin(d), 0.01);
  Sample s;
  ASSERT_TRUE(imu.read(s));
  EXPECT_NEAR(s.accel.x, -9.8 * std::sin(5.0 * M_PI / 180.0), 0.01);
  EXPECT_NEAR(s.accel.y, 0.0, 0.01);
}

TEST(Lsm6dsv, ChipGoneLogsBoundedAndRecovers)
{
  auto chip = std::make_unique<FakeChip>();
  FakeChip * c = chip.get();
  setBodyAccel(*c, 0.0, 0.0, 9.8);
  std::vector<std::string> errors;
  Lsm6dsv imu(std::move(chip), [&errors](int level, const std::string & m) {
    if (level == 2) errors.push_back(m);
  });
  ASSERT_TRUE(imu.init());
  ASSERT_TRUE(imu.calibrateGyroBias(16));
  ASSERT_TRUE(imu.enableFifo(FifoOptions{}));

  // Connection open for 300 reads (6 s at 50 Hz): 100 re-inits, but only the first
  // in full and a summary every 50th - no ERROR flood.
  c->gone = true;
  Sample s;
  for (int i = 0; i < 300; i++) EXPECT_FALSE(imu.read(s));
  EXPECT_EQ(imu.resets(), 100);
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_NE(errors[0].find("WHO_AM_I -1"), std::string::npos) << errors[0];
  EXPECT_NE(errors[1].find("after 50 re-inits"), std::string::npos) << errors[1];
  EXPECT_TRUE(imu.needsCalibration());

  // Back (power returned: registers at reset): re-configured, delivers, still needs a calibration.
  c->gone = false;
  c->regs[0x10] = c->regs[0x11] = c->regs[0x0A] = 0;
  for (int i = 0; i < 3; i++) imu.read(s);
  EXPECT_EQ(c->regs[0x11], 0x06);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.needsCalibration());
  ASSERT_TRUE(imu.calibrateGyroBias(16, 0.1, 0.5));
  EXPECT_FALSE(imu.needsCalibration());

  // The next stall after good samples is logged in full again.
  errors.clear();
  for (int i = 0; i < 3; i++) EXPECT_FALSE(imu.read(s));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("WHO_AM_I 112"), std::string::npos) << errors[0];
}

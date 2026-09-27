// LSM6DSV driver against a fake chip: bias calibration, axis remap, auto-level.
#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <map>

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
  void push(uint8_t tag, int16_t x, int16_t y, int16_t z)
  {
    fifo.push_back({static_cast<uint8_t>(tag << 3), static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8 & 0xFF),
                    static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>(y >> 8 & 0xFF),
                    static_cast<uint8_t>(z & 0xFF), static_cast<uint8_t>(z >> 8 & 0xFF)});
  }
  int readReg8(uint8_t reg) override
  {
    if (reg == 0x12) return regs[reg] & ~0x01;  // reset bit self-clears
    if (reg == 0x1B) return static_cast<int>(fifo.size() & 0xFF);
    if (reg == 0x1C) return static_cast<int>(fifo.size() >> 8 & 0x01);
    return regs.count(reg) ? regs[reg] : 0;
  }
  bool writeReg8(uint8_t reg, uint8_t v) override
  {
    regs[reg] = v;
    return true;
  }
  bool burstRead(uint8_t reg, uint8_t * buf, int len) override
  {
    if (reg == 0x78 && len == 7) {
      if (fifo.empty()) return false;
      std::memcpy(buf, fifo.front().data(), 7);
      fifo.pop_front();
      return true;
    }
    const int16_t * src = reg == 0x22 ? gyro : reg == 0x28 ? accel : nullptr;
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
  ASSERT_TRUE(imu.enableFifo(opt));
  EXPECT_EQ(c->regs[0x0A], 0x06);  // stream mode
  EXPECT_EQ(c->regs[0x09], 0x65);  // gyro 120 Hz, accel 60 Hz
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

  // SFLP bias (4.375 mdps/LSB = gyro LSB / 4) far from the startup bias: not used.
  c->push(0x16, 0, 0, 0);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.hasSflpBias());
  EXPECT_FALSE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);

  // Converged (agrees with the startup bias): tracked from now on.
  c->push(0x16, 40, -80, 20);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.sflpBiasInUse());
  // Drifts a little (Z bias sensor -Y: -20 -> -24 LSB): follows it.
  c->push(0x16, 40, -96, 20);
  c->push(0x01, 10, -24, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_TRUE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
  // Runs away (> 0.5 dps from the startup bias): back to the startup bias.
  c->push(0x16, 40, -2000, 20);
  c->push(0x01, 10, -20, 5);
  ASSERT_TRUE(imu.read(s));
  EXPECT_FALSE(imu.sflpBiasInUse());
  EXPECT_NEAR(s.gyro.z, 0.0, 1e-9);
}

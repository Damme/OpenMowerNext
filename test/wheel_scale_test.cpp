#include "wheel_scale/wheel_scale.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace open_mower_next::wheel_scale;

namespace
{
// Drives the calibrator like the robot: wheels at 50 Hz, RTK at 5 Hz. `truth`
// = real metres per tick metre (1.03: ticks read 3 % short).
struct Sim
{
  Calibrator cal;
  double t = 0, x = 0, y = 0, yaw = 0, truth = 1.03;
  explicit Sim(Params p = {}) : cal(p) {}

  // v: true speed (m/s), w: yaw rate, slip: extra left-wheel tick travel fraction.
  void drive(double seconds, double v, double w = 0.0, double slip = 0.0, bool rtk = true, double gps_noise = 0.0)
  {
    const double dt = 0.02;
    for (int i = 0; i < static_cast<int>(std::lround(seconds / dt)); ++i) {
      t += dt;
      const double d = v * dt;
      x += d * std::cos(yaw);
      y += d * std::sin(yaw);
      yaw += w * dt;
      const double ticks = d / truth;
      const double dw = w * dt * 0.195 / truth;  // half the track
      cal.onWheels(ticks - dw + ticks * slip, ticks + dw, dt);
      cal.onHeading(yaw);
      if (i % 10 == 0) {
        if (!rtk) {
          cal.onBadGps(t);
        } else {
          const double n = gps_noise * std::sin(t * 7.3);
          cal.onGps(t, x + n, y - n);
        }
      }
    }
  }
};
}  // namespace

TEST(WheelScale, ConvergesOnStraightDrives)
{
  Sim s;
  s.drive(3.0, 0.0);  // RTK settles
  for (int k = 0; k < 40; ++k) s.drive(12.0, 0.3);
  EXPECT_GT(s.cal.accepted(), 20);
  EXPECT_NEAR(s.cal.scale(), 1.03, 0.003);
}

TEST(WheelScale, IgnoresTurnsAndSlip)
{
  Sim turning;
  turning.drive(3.0, 0.0);
  for (int k = 0; k < 20; ++k) turning.drive(12.0, 0.3, 0.2);
  EXPECT_EQ(turning.cal.accepted(), 0);
  EXPECT_DOUBLE_EQ(turning.cal.scale(), 1.0);

  Sim slipping;
  slipping.drive(3.0, 0.0);
  for (int k = 0; k < 20; ++k) slipping.drive(12.0, 0.3, 0.0, 0.5);  // one wheel spinning 50 % extra
  EXPECT_EQ(slipping.cal.accepted(), 0);
}

TEST(WheelScale, RtkLossVetoesTheWindow)
{
  Sim s;
  s.drive(3.0, 0.0);
  s.drive(11.0, 0.3);         // a full window...
  s.drive(0.5, 0.3, 0, 0, false);  // ...then RTK drops inside the confirmation horizon
  s.drive(5.0, 0.0);
  EXPECT_EQ(s.cal.accepted(), 0);
  EXPECT_DOUBLE_EQ(s.cal.scale(), 1.0);
}

TEST(WheelScale, StandingStillLearnsNothing)
{
  Sim s;
  s.drive(120.0, 0.0, 0.0, 0.0, true, 0.02);
  EXPECT_EQ(s.cal.seen(), 0);
  EXPECT_DOUBLE_EQ(s.cal.scale(), 1.0);
}

TEST(WheelScale, ClampedAndFiniteStart)
{
  Calibrator c({}, 2.0);
  EXPECT_DOUBLE_EQ(c.scale(), 1.15);
  Calibrator n({}, NAN);
  EXPECT_DOUBLE_EQ(n.scale(), 1.0);
}

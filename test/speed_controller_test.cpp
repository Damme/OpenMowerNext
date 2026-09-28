#include "worx_hardware/speed_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace open_mower_next::worx_hardware;

namespace
{
// Motor like the Worx at crawl speed: nothing below a PWM deadband, then
// linear; first-order lag. The board reports whole ticks (414 per m) every
// 50 ms; the controller runs at 50 Hz.
struct Plant
{
  double deadband = 150, mps_per_pwm = 1.0 / 2500.0, tau = 0.15, v = 0, x = 0;
  bool blocked = false;
  void step(int pwm, double dt)
  {
    const double mag = std::max(0.0, std::abs(pwm) - deadband) * mps_per_pwm;
    const double target = blocked ? 0.0 : (pwm >= 0 ? mag : -mag);
    v += dt / (tau + dt) * (target - v);
    x += v * dt;
  }
  double ticks() const { return std::trunc(x * 414.0) / 414.0; }
};

struct Loop
{
  WheelSpeedPI pi;
  SpeedFilter filt;
  SpeedGains g;
  Plant plant;
  double reported = 0, last_reported = 0;
  int pwm = 0, i = 0;
  static constexpr double dt = 0.02;

  void run(double target, double seconds)
  {
    for (int n = 0; n < static_cast<int>(std::lround(seconds / dt)); ++n, ++i) {
      if (i % 5 == 0) {  // tick report every 0.1 s: twice the board's 50 ms, a worst case
        last_reported = reported;
        reported = plant.ticks();
        filt.update((reported - last_reported) / 0.1, 0.1, 0.25);
      }
      pwm = pi.update(target, reported, filt.value(), dt, g, 1230);
      EXPECT_GE(pwm * (target >= 0 ? 1 : -1), 0) << "PWM against the target direction";
      plant.step(pwm, dt);
    }
  }
};
}  // namespace

TEST(SpeedController, ReachesCrawlSpeedDespiteDeadband)
{
  Loop fwd;
  fwd.run(0.15, 3.0);
  EXPECT_NEAR(fwd.plant.v, 0.15, 0.015);
  Loop rev;
  rev.run(-0.15, 3.0);
  EXPECT_NEAR(rev.plant.v, -0.15, 0.015);
  Loop fast;
  fast.run(0.30, 3.0);
  EXPECT_NEAR(fast.plant.v, 0.30, 0.02);
}

TEST(SpeedController, TracksDistanceFromTicks)
{
  // The commanded distance is what the wheel covers (within the start-up lag and
  // a tick or two), unlike a speed loop on noisy tick rates.
  Loop l;
  l.run(0.15, 10.0);
  EXPECT_NEAR(l.plant.x, 1.5, 0.03);
  Loop weak;  // weaker motor (slope, long grass): the integral makes up the difference
  weak.plant.mps_per_pwm = 1.0 / 3500.0;
  weak.run(0.15, 10.0);
  EXPECT_NEAR(weak.plant.x, 1.5, 0.05);
}

TEST(SpeedController, NoCatchUpSurgeAfterBlockedWheel)
{
  Loop l;
  l.run(0.15, 2.0);
  l.plant.blocked = true;
  l.run(0.15, 2.0);  // 0.3 m of reference lost...
  EXPECT_LE(std::abs(l.pi.error()), l.g.pos_max + 1e-9);
  l.plant.blocked = false;
  double peak = 0;
  for (int k = 0; k < 100; ++k) {
    l.run(0.15, 0.02);
    peak = std::max(peak, l.plant.v);
  }
  EXPECT_LT(peak, 0.30);  // ...is not driven back in: at most a short push
  EXPECT_NEAR(l.plant.v, 0.15, 0.02);
}

TEST(SpeedController, ZeroTargetStopsAtOnce)
{
  WheelSpeedPI pi;
  SpeedGains g;
  for (int i = 0; i < 100; ++i) pi.update(0.2, 0.0, 0.0, 0.01, g, 1230);
  EXPECT_GT(pi.integral(), 0);
  EXPECT_EQ(pi.update(0.0, 0.0, 0.2, 0.01, g, 1230), 0);
  EXPECT_EQ(pi.integral(), 0);
}

TEST(SpeedController, IntegralBoundedWhenStalled)
{
  WheelSpeedPI pi;
  SpeedGains g;
  int pwm = 0;
  for (int i = 0; i < 2000; ++i) pwm = pi.update(0.15, 0.0, 0.0, 0.01, g, 1230);  // wheel blocked 20 s
  EXPECT_LE(pi.integral(), g.i_max);
  EXPECT_LE(pwm, 1230);
}

TEST(SpeedController, DirectionChangeRestartsTracking)
{
  WheelSpeedPI pi;
  SpeedGains g;
  for (int i = 0; i < 100; ++i) pi.update(0.15, 0.0, 0.0, 0.02, g, 1230);
  const int pwm = pi.update(-0.15, 0.0, 0.0, 0.02, g, 1230);
  EXPECT_LT(pwm, 0);
  EXPECT_LE(pi.integral(), 0);
}

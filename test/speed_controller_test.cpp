#include "worx_hardware/speed_controller.hpp"

#include <gtest/gtest.h>

using namespace open_mower_next::worx_hardware;

namespace
{
// Motor like the Worx at crawl speed: nothing below a PWM deadband, then
// linear; first-order lag. Speed is sampled every 50 ms (the board's reports),
// the controller runs at 100 Hz.
struct Plant
{
  double deadband = 150, mps_per_pwm = 1.0 / 1230.0, tau = 0.15, v = 0;
  void step(int pwm, double dt)
  {
    const double mag = std::max(0.0, std::abs(pwm) - deadband) * mps_per_pwm;
    const double target = pwm >= 0 ? mag : -mag;
    v += dt / (tau + dt) * (target - v);
  }
};

double run(double target, Plant & plant, double seconds, int * last_pwm = nullptr)
{
  WheelSpeedPI pi;
  SpeedFilter filt;
  SpeedGains g;
  const double dt = 0.01;
  int pwm = 0;
  for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
    if (i % 5 == 0) filt.update(plant.v, 0.05, 0.25);
    pwm = pi.update(target, filt.value(), dt, g, 1230);
    EXPECT_GE(pwm * (target >= 0 ? 1 : -1), 0) << "PWM against the target direction";
    plant.step(pwm, dt);
  }
  if (last_pwm) *last_pwm = pwm;
  return plant.v;
}
}  // namespace

TEST(SpeedController, ReachesCrawlSpeedDespiteDeadband)
{
  Plant plant;
  int pwm = 0;
  EXPECT_NEAR(run(-0.15, plant, 3.0, &pwm), -0.15, 0.015);
  EXPECT_LT(pwm, -184);  // more than the open-loop feedforward
  Plant p2;
  EXPECT_NEAR(run(0.30, p2, 3.0), 0.30, 0.02);
}

TEST(SpeedController, ZeroTargetStopsAtOnce)
{
  WheelSpeedPI pi;
  SpeedGains g;
  for (int i = 0; i < 100; ++i) pi.update(0.2, 0.0, 0.01, g, 1230);
  EXPECT_GT(pi.integral(), 0);
  EXPECT_EQ(pi.update(0.0, 0.2, 0.01, g, 1230), 0);
  EXPECT_EQ(pi.integral(), 0);
}

TEST(SpeedController, IntegralBoundedWhenStalled)
{
  WheelSpeedPI pi;
  SpeedGains g;
  int pwm = 0;
  for (int i = 0; i < 2000; ++i) pwm = pi.update(0.15, 0.0, 0.01, g, 1230);  // wheel blocked 20 s
  EXPECT_LE(pi.integral(), g.i_max);
  EXPECT_LE(pwm, 1230);
}

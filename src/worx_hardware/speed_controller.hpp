// Wheel speed control for the Worx board. The firmware takes open-loop PWM, and
// the motors barely turn at low PWM (0.15 m/s -> PWM 184 moved the robot at
// ~0.03 m/s): feedforward pwm_per_mps * v plus a PI on the measured wheel speed.
// No ROS here (unit-tested).
#pragma once

#include <algorithm>
#include <cmath>

namespace open_mower_next::worx_hardware
{

struct SpeedGains
{
  double ff = 1230.0;    // PWM per m/s (ROS1 MAXSPEED)
  double kp = 1500.0;    // PWM per m/s of error
  double ki = 3000.0;    // PWM per (m/s * s) of error
  double i_max = 800.0;  // |integral| limit, PWM
};

class WheelSpeedPI
{
public:
  // target, measured: m/s (signed). Returns PWM in [-max_pwm, max_pwm], never
  // opposite to the target (no active braking), 0 for a zero target.
  int update(double target, double measured, double dt, const SpeedGains & g, int max_pwm)
  {
    if (!std::isfinite(target) || target == 0.0 || !std::isfinite(measured) || dt <= 0.0) {
      reset();
      return 0;
    }
    const double dir = target > 0 ? 1.0 : -1.0;
    if (integral_ * dir < 0) integral_ = 0.0;  // direction changed
    const double err = target - measured;
    const double p = g.ff * target + g.kp * err;
    const double candidate = std::clamp(integral_ + g.ki * err * dt, -g.i_max, g.i_max);
    // Anti-windup: only integrate while the output isn't saturated in that direction.
    if (std::abs(p + candidate) <= max_pwm || std::abs(candidate) < std::abs(integral_)) integral_ = candidate;
    double out = std::clamp(p + integral_, -static_cast<double>(max_pwm), static_cast<double>(max_pwm));
    if (out * dir < 0) out = 0.0;
    return static_cast<int>(out);
  }

  void reset() { integral_ = 0.0; }
  double integral() const { return integral_; }

private:
  double integral_ = 0.0;
};

// First-order low-pass for the measured wheel speed: the board reports ticks at
// 20 Hz and one tick is ~2.4 mm, so single reports are 0 or ~0.05 m/s at crawl speed.
class SpeedFilter
{
public:
  double update(double v, double dt, double tau)
  {
    if (!std::isfinite(v)) return value_;
    if (dt <= 0.0 || dt > 1.0) {
      value_ = v;
    } else {
      value_ += dt / (tau + dt) * (v - value_);
    }
    return value_;
  }
  double value() const { return value_; }
  void reset() { value_ = 0.0; }

private:
  double value_ = 0.0;
};

}  // namespace open_mower_next::worx_hardware

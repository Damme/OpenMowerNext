// Wheel speed control for the Worx board. The firmware takes open-loop PWM, and
// the motors barely turn at low PWM (0.15 m/s -> PWM 184 moved the robot at
// ~0.03 m/s). Each wheel tracks the DISTANCE its commanded speed integrates to,
// measured in wheel ticks (Daniel: use the ticks, not the speed): at crawl speed
// a 50 ms report holds only 2-3 ticks, so a speed from ticks/dt is +-33 % noise,
// while the tick count itself is exact. Output = feedforward (static + slope)
// + P and I on the distance error + a little damping on the speed error.
// The distance error is clamped (pos_max): after a slip or a blocked wheel the
// reference is pulled along instead of forcing a catch-up surge.
// No ROS here (unit-tested).
#pragma once

#include <algorithm>
#include <cmath>

namespace open_mower_next::worx_hardware
{

struct SpeedGains
{
  // Feedforward |pwm| = ff_static + ff * |v|. Measured on grass 2026-09-28:
  // ~490 PWM at 0.15 m/s (straight, reverse and spin alike), 184 PWM -> ~0.03 m/s.
  double ff = 2500.0;        // PWM per m/s
  double ff_static = 110.0;  // PWM to get the wheel turning
  double kp = 4000.0;        // PWM per m of distance error
  double ki = 4000.0;        // PWM per (m * s) of distance error
  double kv = 500.0;         // PWM per m/s of speed error (damping)
  double i_max = 800.0;      // |integral| limit, PWM
  double pos_max = 0.03;     // m: max distance error the loop tries to make up
};

class WheelSpeedPI
{
public:
  // target: m/s (signed). position: the wheel's travel in m (from the ticks,
  // signed, any origin). speed: measured m/s (low-passed; only for damping).
  // Returns PWM in [-max_pwm, max_pwm], never opposite to the target (no active
  // braking), 0 for a zero target.
  int update(double target, double position, double speed, double dt, const SpeedGains & g, int max_pwm)
  {
    if (!std::isfinite(target) || target == 0.0 || !std::isfinite(position) || dt <= 0.0) {
      reset();
      return 0;
    }
    const double dir = target > 0 ? 1.0 : -1.0;
    if (!active_ || dir != dir_) {  // start or direction change: track from here
      active_ = true;
      dir_ = dir;
      ref_ = position;
      integral_ = 0.0;
    }
    ref_ += target * dt;
    double err = ref_ - position;
    if (std::abs(err) > g.pos_max) {
      err = std::clamp(err, -g.pos_max, g.pos_max);
      ref_ = position + err;
    }
    err_ = err;
    const double v_err = std::isfinite(speed) ? target - speed : 0.0;
    const double p = dir * g.ff_static + g.ff * target + g.kp * err + g.kv * v_err;
    const double candidate = std::clamp(integral_ + g.ki * err * dt, -g.i_max, g.i_max);
    // Anti-windup: only integrate while the output isn't saturated in that direction.
    if (std::abs(p + candidate) <= max_pwm || std::abs(candidate) < std::abs(integral_)) integral_ = candidate;
    double out = std::clamp(p + integral_, -static_cast<double>(max_pwm), static_cast<double>(max_pwm));
    if (out * dir < 0) out = 0.0;
    return static_cast<int>(out);
  }

  void reset()
  {
    active_ = false;
    integral_ = 0.0;
    err_ = 0.0;
  }
  double integral() const { return integral_; }
  double error() const { return err_; }  // m, last distance error

private:
  bool active_ = false;
  double dir_ = 0.0;
  double ref_ = 0.0;
  double err_ = 0.0;
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

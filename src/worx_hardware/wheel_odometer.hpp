#pragma once
// Wheel travel from the Worx MotorPulse counters.
//
// The firmware reports a cumulative tick MAGNITUDE per wheel plus a direction
// bit; the delta since the last message takes the sign of the current bit
// (same as xbot_positioning's onWheelTicks in the ROS1 stack).

#include <cstdint>

namespace open_mower_next::worx_hardware
{

class WheelOdometer
{
public:
  // max_delta_ticks: a jump larger than this (counter reset, garbage) is
  // ignored and re-baselines the counter.
  explicit WheelOdometer(double ticks_per_m = 414.0, int counter_bits = 32, uint64_t max_delta_ticks = 2000)
  : ticks_per_m_(ticks_per_m),
    mask_(counter_bits >= 64 ? ~0ULL : ((1ULL << counter_bits) - 1)),
    max_delta_(max_delta_ticks)
  {
  }

  // Returns the signed travel [m] since the previous update (0 on the first).
  double update(uint64_t ticks, bool reverse)
  {
    ticks &= mask_;
    if (!initialized_) {
      initialized_ = true;
      last_ = ticks;
      return 0.0;
    }
    const uint64_t delta = (ticks - last_) & mask_;
    last_ = ticks;
    if (delta > max_delta_) {
      ++glitches_;
      return 0.0;
    }
    const double d = static_cast<double>(delta) / ticks_per_m_ * (reverse ? -1.0 : 1.0);
    distance_ += d;
    return d;
  }

  double distance() const { return distance_; }
  uint32_t glitches() const { return glitches_; }

private:
  double ticks_per_m_;
  uint64_t mask_;
  uint64_t max_delta_;
  bool initialized_ = false;
  uint64_t last_ = 0;
  double distance_ = 0.0;
  uint32_t glitches_ = 0;
};

}  // namespace open_mower_next::worx_hardware

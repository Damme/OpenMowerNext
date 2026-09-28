// Adaptive wheel scale (metres per tick distance) from RTK geometry - a port of the
// ROS1 xbot_positioning wheel calibration (Daniel's fork) that the Worx mowed with.
// Grass height, debris in the treads and wear change the rolling radius: measured
// 2026-09-28 the ticks read 0.1-3.5 % short of RTK on one lawn within an hour.
//
// Not from speed ratios (|noisy GPS velocity| is biased high): a window collects
// min_dist of wheel travel, split in two halves, and compares the straight-line
// RTK displacement with the summed wheel distance. Accepted only if the heading
// (gyro) stayed within max_hdg, the three RTK points are collinear, both halves
// agree, left/right asymmetry is low (no slip/turning) and the ratio is plausible;
// then held back until RTK stayed good for confirm_time after it (a late RTK loss
// still vetoes it), and applied as an EMA (rate) with a hard clamp.
// No ROS here (unit-tested).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

namespace open_mower_next::wheel_scale
{

struct Params
{
  double rate = 0.10;          // EMA step per accepted window
  double min_speed = 0.10;     // m/s wheel speed to count as moving
  double min_dist = 3.0;       // m of wheel travel per window (two halves)
  double agree_tol = 0.05;     // max |ratio A - ratio B|
  double straight_tol = 0.03;  // (gA + gB) <= gT * (1 + tol)
  double max_hdg = 0.10;       // rad heading change over the window
  double max_asym = 0.20;      // sum |dL - dR| / distance
  double min_scale = 0.85, max_scale = 1.15;
  double settle_time = 2.0;    // s of good RTK before a window may start
  double confirm_time = 2.0;   // s of good RTK after it before it counts
  double gps_gap = 1.0;        // s without an RTK sample = RTK lost
};

class Calibrator
{
public:
  explicit Calibrator(Params p = {}, double scale = 1.0) : p_(p), scale_(clampScale(scale)) {}

  double scale() const { return scale_; }
  int seen() const { return seen_; }
  int accepted() const { return accepted_; }
  const std::string & lastMessage() const { return msg_; }

  // Raw (unscaled) wheel travel since the last call [m], signed, and its time span.
  void onWheels(double dl, double dr, double dt)
  {
    if (!(dt > 0.0)) return;
    const double d = 0.5 * (dl + dr);
    time_ += dt;  // speed over the time since the last GPS sample: the 20 Hz tick
    dist_ += std::abs(d);  // reports make single 50 Hz samples 0 or double
    asym_ += std::abs(dl - dr);
  }

  void onHeading(double yaw) { yaw_ = yaw; }

  // RTK is not fixed / GPS is gated off right now (drops any window in flight).
  void onBadGps(double t)
  {
    last_bad_ = t;
    active_ = false;
    pending_.reset();
  }

  // One RTK-fixed position (map frame). Returns true when the scale changed.
  bool onGps(double t, double x, double y)
  {
    msg_.clear();
    if (last_gps_ && t - *last_gps_ > p_.gps_gap) onBadGps(t);
    last_gps_ = t;
    const double d_w = dist_, d_asym = asym_;
    const double speed = time_ > 0.0 ? d_w / time_ : 0.0;
    dist_ = asym_ = time_ = 0.0;

    const bool settled = t - last_bad_ > p_.settle_time;
    const bool moving = speed > p_.min_speed;
    if (!settled || !moving) {
      active_ = false;
    } else if (!active_) {
      active_ = true;
      half_ = 0;
      da_ = db_ = wasym_ = 0.0;
      x0_ = x;
      y0_ = y;
      h0_ = yaw_;
      t0_ = t;
    } else if (half_ == 0) {
      da_ += d_w;
      wasym_ += d_asym;
      if (da_ >= 0.5 * p_.min_dist) {
        half_ = 1;
        xm_ = x;
        ym_ = y;
      }
    } else {
      db_ += d_w;
      wasym_ += d_asym;
      if (da_ + db_ >= p_.min_dist) judge(t, x, y);
    }

    bool changed = false;
    if (pending_ && t - pending_->t1 > p_.confirm_time) {
      if (pending_->t0 - last_bad_ > p_.settle_time) {
        const double before = scale_;
        scale_ = clampScale(scale_ + p_.rate * (pending_->ratio - scale_));
        ++accepted_;
        changed = scale_ != before;
        msg_ = "accepted ratio " + fmt(pending_->ratio) + ", scale " + fmt(before) + " -> " + fmt(scale_);
      } else {
        msg_ = "RTK degraded in the confirmation horizon: discarded";
      }
      pending_.reset();
    }
    return changed;
  }

private:
  struct Pending
  {
    double ratio, t0, t1;
  };

  void judge(double t, double x, double y)
  {
    const double gA = std::hypot(xm_ - x0_, ym_ - y0_);
    const double gB = std::hypot(x - xm_, y - ym_);
    const double gT = std::hypot(x - x0_, y - y0_);
    const double rA = gA / std::max(da_, 1e-6);
    const double rB = gB / std::max(db_, 1e-6);
    const double r = gT / std::max(da_ + db_, 1e-6);
    const double dh = std::remainder(yaw_ - h0_, 2.0 * M_PI);
    const bool turned = std::abs(dh) > p_.max_hdg;
    const bool straight = gA + gB <= gT * (1.0 + p_.straight_tol);
    const bool agree = std::abs(rA - rB) <= p_.agree_tol;
    const bool no_slip = wasym_ <= p_.max_asym * std::max(da_ + db_, 1e-6);
    const bool sane = r > p_.min_scale && r < p_.max_scale;
    ++seen_;
    if (!turned && straight && agree && no_slip && sane) {
      pending_ = Pending{r, t0_, t};
      msg_ = "window ok r=" + fmt(r) + " (halves " + fmt(rA) + " " + fmt(rB) + "), confirming";
    } else {
      msg_ = std::string("window rejected:") + (turned ? " turning" : "") + (!straight ? " not-straight" : "") +
             (!agree ? " halves-disagree" : "") + (!no_slip ? " slip" : "") + (!sane ? " out-of-range" : "") +
             " (r=" + fmt(r) + ")";
    }
    active_ = false;
  }

  double clampScale(double s) const { return std::clamp(std::isfinite(s) ? s : 1.0, p_.min_scale, p_.max_scale); }
  static std::string fmt(double v)
  {
    char b[32];
    std::snprintf(b, sizeof b, "%.4f", v);
    return b;
  }

  Params p_;
  double scale_;
  double time_ = 0.0, dist_ = 0.0, asym_ = 0.0, yaw_ = 0.0;
  double last_bad_ = -1e9;
  std::optional<double> last_gps_;
  bool active_ = false;
  int half_ = 0;
  double da_ = 0, db_ = 0, wasym_ = 0, x0_ = 0, y0_ = 0, xm_ = 0, ym_ = 0, h0_ = 0, t0_ = 0;
  std::optional<Pending> pending_;
  int seen_ = 0, accepted_ = 0;
  std::string msg_;
};

}  // namespace open_mower_next::wheel_scale

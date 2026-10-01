#include "docking_helper/line_docker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer_interface.h>

namespace open_mower_next::docking_helper
{
namespace
{
double norm(double a) { return std::atan2(std::sin(a), std::cos(a)); }

bool inside(double x, double y, const std::vector<geometry_msgs::msg::Point32> & poly)
{
  bool c = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto & a = poly[i];
    const auto & b = poly[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) c = !c;
  }
  return c;
}

const char * driveText(int d)
{
  static const char * t[] = {"reached", "contact", "collision", "overshoot", "timeout", "no pose", "cancelled"};
  return t[d];
}
}  // namespace

LineDocker::LineDocker(rclcpp::Node & node, std::shared_ptr<tf2_ros::Buffer> tf, std::function<void(bool)> dock_zone)
: node_(node), tf_(std::move(tf)), dock_zone_(std::move(dock_zone))
{
  auto d = [&](const std::string & n, double v) { return node_.declare_parameter("line_dock." + n, v); };
  p_.approach_length = d("approach_length", p_.approach_length);
  p_.gate_margin = d("gate_margin", p_.gate_margin);
  p_.gate_fallback = d("gate_fallback", p_.gate_fallback);
  p_.speed = d("speed", p_.speed);
  p_.push_speed = d("push_speed", p_.push_speed);
  p_.reverse_speed = d("reverse_speed", p_.reverse_speed);
  p_.k_lateral = d("k_lateral", p_.k_lateral);
  p_.k_heading = d("k_heading", p_.k_heading);
  p_.max_angular = d("max_angular", p_.max_angular);
  p_.lateral_tolerance = d("lateral_tolerance", p_.lateral_tolerance);
  p_.heading_tolerance = d("heading_tolerance_deg", p_.heading_tolerance * 180.0 / M_PI) * M_PI / 180.0;
  p_.require_gps = node_.declare_parameter("line_dock.require_gps", p_.require_gps);
  p_.gps_window = d("gps_window", p_.gps_window);
  p_.gps_max_gap = d("gps_max_gap", p_.gps_max_gap);
  p_.gps_heading_tolerance =
    d("gps_heading_tolerance_deg", p_.gps_heading_tolerance * 180.0 / M_PI) * M_PI / 180.0;
  p_.gps_position_tolerance = d("gps_position_tolerance", p_.gps_position_tolerance);
  p_.gps_wait = d("gps_wait", p_.gps_wait);
  p_.push_overshoot = d("push_overshoot", p_.push_overshoot);
  p_.backoff = d("backoff", p_.backoff);
  p_.charge_wait = d("charge_wait", p_.charge_wait);
  p_.attempts = static_cast<int>(node_.declare_parameter("line_dock.attempts", p_.attempts));
  p_.nav_timeout = d("nav_timeout", p_.nav_timeout);

  cmd_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
    node_.declare_parameter("line_dock.cmd_vel_topic", std::string("/cmd_vel_nav")), 10);
  if (p_.require_gps) {
    gps_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      node_.declare_parameter("line_dock.gps_odom_topic", std::string("/odometry/gps")), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mutex_);
        gps_.push_back({rclcpp::Time(m->header.stamp, RCL_ROS_TIME), m->pose.pose.position.x,
                        m->pose.pose.position.y});
        while (gps_.size() > 200) gps_.pop_front();
      });
  }
  status_sub_ = node_.create_subscription<open_mower_next::msg::WorxStatus>(
    "/worx/status", 10, [this](open_mower_next::msg::WorxStatus::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      status_ = m;
      status_time_ = node_.now();
    });
  charger_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
    "/power/charger_present", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      charger_ = m->data;
    });
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(&node_, "/navigate_to_pose");
}

double LineDocker::areaEdge(const geometry_msgs::msg::Pose & dock_pose,
                            const std::vector<open_mower_next::msg::Area> & areas, double max_s)
{
  const double yaw = tf2::getYaw(dock_pose.orientation);
  for (double s = 0.0; s <= max_s; s += 0.05) {
    const double x = dock_pose.position.x - s * std::cos(yaw);
    const double y = dock_pose.position.y - s * std::sin(yaw);
    bool in_area = false, excluded = false;
    for (const auto & a : areas) {
      if (!inside(x, y, a.area.polygon.points)) continue;
      if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION) excluded = true;
      else in_area = true;
    }
    if (in_area && !excluded) return s;
  }
  return NAN;
}

bool LineDocker::linePose(LinePose & lp) const
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  lp.stamp = rclcpp::Time(t.header.stamp, RCL_ROS_TIME);
  if ((node_.now() - lp.stamp).seconds() > 0.5) return false;
  const double dx = t.transform.translation.x - dock_x_;
  const double dy = t.transform.translation.y - dock_y_;
  const double c = std::cos(dock_yaw_), s = std::sin(dock_yaw_);
  lp.s = -(dx * c + dy * s);
  lp.lateral = -s * dx + c * dy;
  lp.yaw = tf2::getYaw(t.transform.rotation);
  lp.heading = norm(lp.yaw - dock_yaw_);
  return true;
}

void LineDocker::publish(double v, double w)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_.now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;
  cmd_pub_->publish(cmd);
}

void LineDocker::stop()
{
  for (int i = 0; i < 3; ++i) publish(0.0, 0.0);
}

bool LineDocker::charger() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return charger_;
}

void LineDocker::setDockZone(bool on)
{
  if (on == in_dock_zone_) return;
  in_dock_zone_ = on;
  RCLCPP_INFO(node_.get_logger(), "Line docking: %s the dock zone (GPS %s, docking mode %s)",
              on ? "entering" : "leaving", on ? "off" : "on", on ? "on" : "off");
  if (dock_zone_) dock_zone_(on);
}

bool LineDocker::sleepFor(double seconds, const std::function<bool()> & cancelled)
{
  const auto until = node_.now() + rclcpp::Duration::from_seconds(seconds);
  while (node_.now() < until) {
    if (cancelled()) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return true;
}

LineDocker::Drive LineDocker::driveTo(double target, double speed, Steer steer, bool stop_on_contact,
                                      double timeout, const std::function<bool()> & cancelled,
                                      const std::function<void(const LinePose &)> & on_tick)
{
  LinePose lp;
  if (!linePose(lp)) return Drive::NO_POSE;
  const bool forward = target < lp.s;
  const double start_s = lp.s;
  const rclcpp::Time start = node_.now();
  rclcpp::Time last_pose = start;
  rclcpp::Time moved_at{0, 0, RCL_ROS_TIME};  // first moved 2 cm (BlockForward counts after that)
  std::deque<std::pair<rclcpp::Time, double>> track;  // stall detection
  std::string start_power;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (status_) start_power = status_->power_state;
  }
  while (rclcpp::ok()) {
    const rclcpp::Time now = node_.now();
    if (cancelled()) {
      stop();
      return Drive::CANCELLED;
    }
    if (!linePose(lp)) {
      publish(0.0, 0.0);
      if ((now - last_pose).seconds() > 1.0) {
        stop();
        return Drive::NO_POSE;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    last_pose = now;
    if (on_tick) on_tick(lp);
    const double remaining = forward ? lp.s - target : target - lp.s;
    if (remaining <= 0.0) {
      stop();
      return forward && target < 0.0 ? Drive::OVERSHOOT : Drive::REACHED;
    }

    open_mower_next::msg::WorxStatus::ConstSharedPtr st;
    rclcpp::Time st_time{0, 0, RCL_ROS_TIME};
    bool chg;
    {
      std::lock_guard<std::mutex> l(mutex_);
      st = status_;
      st_time = status_time_;
      chg = charger_;
    }
    const bool st_fresh = st && (now - st_time).seconds() < 1.0;
    if (forward && !in_dock_zone_ && st_fresh && st->collision) {
      stop();
      return Drive::COLLISION;
    }
    if (stop_on_contact) {
      const double progress = std::abs(lp.s - start_s);
      if (moved_at.nanoseconds() == 0 && progress > 0.02) moved_at = now;
      track.emplace_back(now, lp.s);
      while (track.size() > 1 && (now - track.front().first).seconds() > 1.0) track.pop_front();
      std::string why;
      if (chg) {
        why = "charger present";
      } else if (st_fresh && st_time > start && st->power_state != start_power &&
                 (st->power_state == "StartCharging" || st->power_state == "Charging")) {
        why = "powerState " + st->power_state;
      } else if (st_fresh && moved_at.nanoseconds() != 0 && st_time > moved_at && st->block_forward == 1) {
        why = "bumper (BlockForward)";
      } else if ((now - start).seconds() > 1.5 && (now - track.front().first).seconds() > 0.9 &&
                 std::abs(lp.s - track.front().second) < 0.01) {
        why = "stalled";
      }
      if (!why.empty()) {
        stop();
        RCLCPP_INFO(node_.get_logger(), "Line docking: contact (%s) at s=%.3f lateral=%+.3f heading=%+.1f deg",
                    why.c_str(), lp.s, lp.lateral, lp.heading * 180.0 / M_PI);
        return Drive::CONTACT;
      }
    }
    if ((now - start).seconds() > timeout) {
      stop();
      return Drive::TIMEOUT;
    }

    double desired = 0.0;  // heading relative to the line
    if (steer == Steer::LINE) {
      desired = forward ? -std::atan(p_.k_lateral * lp.lateral) : std::atan(p_.k_lateral * lp.lateral);
    }
    const double w = std::clamp(p_.k_heading * norm(desired - lp.heading), -p_.max_angular, p_.max_angular);
    const double v = std::min(speed, std::max(0.05, remaining * 0.8));
    publish(forward ? v : -v, w);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  stop();
  return Drive::CANCELLED;
}

bool LineDocker::gateCheck(const LinePose & g, const rclcpp::Time & window_start, const std::vector<double> & yaws,
                           std::string & why)
{
  char buf[256];
  std::vector<std::string> bad;
  if (std::abs(g.lateral) > p_.lateral_tolerance) {
    snprintf(buf, sizeof(buf), "lateral %+.3f m", g.lateral);
    bad.emplace_back(buf);
  }
  if (std::abs(g.heading) > p_.heading_tolerance) {
    snprintf(buf, sizeof(buf), "heading %+.1f deg", g.heading * 180.0 / M_PI);
    bad.emplace_back(buf);
  }
  std::string gps_info = "GPS not required";
  if (p_.require_gps) {
    std::vector<GpsSample> w;
    {
      std::lock_guard<std::mutex> l(mutex_);
      for (const auto & s : gps_)
        if (s.stamp >= window_start) w.push_back(s);
    }
    const rclcpp::Time now = node_.now();
    double max_gap = w.empty() ? (now - window_start).seconds() : (w.front().stamp - window_start).seconds();
    for (size_t i = 1; i < w.size(); ++i) max_gap = std::max(max_gap, (w[i].stamp - w[i - 1].stamp).seconds());
    if (!w.empty()) max_gap = std::max(max_gap, (now - w.back().stamp).seconds());
    double track_len = 0.0, track_err = NAN, pos_err = NAN;
    if (w.size() >= 3) {
      const double dx = w.back().x - w.front().x, dy = w.back().y - w.front().y;
      track_len = std::hypot(dx, dy);
      double sx = 0.0, sy = 0.0;
      for (double y : yaws) {
        sx += std::cos(y);
        sy += std::sin(y);
      }
      if (!yaws.empty()) track_err = norm(std::atan2(dy, dx) - std::atan2(sy, sx));
      // EKF vs GPS at the GPS stamps (the EKF trails a moving fix otherwise).
      pos_err = 0.0;
      int n = 0;
      for (size_t i = w.size() >= 3 ? w.size() - 3 : 0; i < w.size(); ++i) {
        try {
          const auto t = tf_->lookupTransform("map", "base_link", tf2_ros::fromRclcpp(w[i].stamp));
          pos_err = std::max(pos_err, std::hypot(t.transform.translation.x - w[i].x, t.transform.translation.y - w[i].y));
          ++n;
        } catch (const tf2::TransformException &) {
        }
      }
      if (n == 0) pos_err = NAN;
    }
    snprintf(buf, sizeof(buf), "%zu fixes, max gap %.2f s, track %.2f m, track vs EKF heading %+.1f deg, "
             "EKF vs GPS %.3f m", w.size(), max_gap, track_len, track_err * 180.0 / M_PI, pos_err);
    gps_info = buf;
    if (w.size() < 3 || max_gap > p_.gps_max_gap) bad.emplace_back("RTK fixed not continuous");
    if (track_len < 0.5 * p_.gps_window) bad.emplace_back("GPS track too short");
    if (!(std::abs(track_err) <= p_.gps_heading_tolerance)) bad.emplace_back("GPS track != EKF heading");
    if (!(pos_err <= p_.gps_position_tolerance)) bad.emplace_back("EKF != GPS position");
  }
  RCLCPP_INFO(node_.get_logger(), "Line docking gate: lateral %+.3f m, heading %+.1f deg; %s", g.lateral,
              g.heading * 180.0 / M_PI, gps_info.c_str());
  why.clear();
  for (const auto & b : bad) why += (why.empty() ? "" : ", ") + b;
  return bad.empty();
}

bool LineDocker::waitGps(double timeout, const std::function<bool()> & cancelled)
{
  if (!p_.require_gps) return true;
  const rclcpp::Time start = node_.now();
  rclcpp::Time fresh_since{0, 0, RCL_ROS_TIME};
  while (rclcpp::ok() && !cancelled()) {
    const rclcpp::Time now = node_.now();
    bool fresh;
    {
      std::lock_guard<std::mutex> l(mutex_);
      fresh = !gps_.empty() && (now - gps_.back().stamp).seconds() < p_.gps_max_gap;
    }
    if (!fresh) fresh_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
    else if (fresh_since.nanoseconds() == 0) fresh_since = now;
    else if ((now - fresh_since).seconds() >= 2.0) return true;
    if ((now - start).seconds() > timeout) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool LineDocker::waitCharge(double timeout, const std::function<bool()> & cancelled)
{
  const rclcpp::Time start = node_.now();
  while (rclcpp::ok() && !cancelled()) {
    if (charger()) return true;
    if ((node_.now() - start).seconds() > timeout) return false;
    publish(0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool LineDocker::navigateToStart(const geometry_msgs::msg::Pose & start, const std::function<bool()> & cancelled)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_.get_logger(), "Line docking: navigate_to_pose not available");
    return false;
  }
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_.now();
  goal.pose.pose = start;
  auto gh_future = nav_client_->async_send_goal(goal);
  const rclcpp::Time t0 = node_.now();
  while (gh_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (cancelled() || (node_.now() - t0).seconds() > 10.0) return false;
  }
  auto gh = gh_future.get();
  if (!gh) {
    RCLCPP_ERROR(node_.get_logger(), "Line docking: start point goal rejected");
    return false;
  }
  auto res_future = nav_client_->async_get_result(gh);
  while (res_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (cancelled() || (node_.now() - t0).seconds() > p_.nav_timeout) {
      nav_client_->async_cancel_goal(gh);
      return false;
    }
  }
  return res_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

LineDocker::Result LineDocker::dock(const geometry_msgs::msg::Pose & dock_pose,
                                    const std::vector<open_mower_next::msg::Area> & areas,
                                    const FeedbackFn & feedback, const std::function<bool()> & cancelled)
{
  dock_x_ = dock_pose.position.x;
  dock_y_ = dock_pose.position.y;
  dock_yaw_ = tf2::getYaw(dock_pose.orientation);
  const double edge = areaEdge(dock_pose, areas, 10.0);
  const double gate = std::isnan(edge) ? p_.gate_fallback : edge + p_.gate_margin;
  const double start_s = gate + p_.approach_length;
  RCLCPP_INFO(node_.get_logger(),
              "Line docking to (%.2f, %.2f, %.0f deg): mow area edge %.2f m, gate %.2f m, start %.2f m out",
              dock_x_, dock_y_, dock_yaw_ * 180.0 / M_PI, edge, gate, start_s);

  geometry_msgs::msg::Pose start = dock_pose;
  start.position.x -= start_s * std::cos(dock_yaw_);
  start.position.y -= start_s * std::sin(dock_yaw_);

  Result res;
  struct Cleanup
  {
    LineDocker * self;
    ~Cleanup()
    {
      self->stop();
      self->setDockZone(false);
    }
  } cleanup{this};

  if (charger()) {
    res.success = true;
    res.message = "Already on the charger";
    return res;
  }

  auto cancelledResult = [&]() {
    res.code = 999;
    res.message = "Docking canceled";
    return res;
  };
  // Straight back along the line to the start point (GPS back on once past the gate).
  auto backOut = [&](int attempt) {
    feedback(BACK_OUT, attempt, "Backing out to the start point");
    LinePose lp;
    if (!linePose(lp)) return false;
    setDockZone(lp.s < gate - 0.05);  // a failed gate check stands right at the gate
    const auto d = driveTo(start_s, p_.reverse_speed, Steer::LINE, false,
                           (start_s - lp.s) / p_.reverse_speed * 2.0 + 15.0, cancelled,
                           [&](const LinePose & now) {
                             if (now.s >= gate) setDockZone(false);
                           });
    if (d != Drive::REACHED) {
      RCLCPP_WARN(node_.get_logger(), "Line docking: backing out ended: %s", driveText(static_cast<int>(d)));
      return false;
    }
    return true;
  };

  for (int attempt = 1; attempt <= p_.attempts; ++attempt) {
    res.attempts = attempt;
    RCLCPP_INFO(node_.get_logger(), "Line docking: attempt %d/%d", attempt, p_.attempts);
    LinePose lp;
    const bool have = linePose(lp);
    const bool on_line = have && lp.s > -0.3 && lp.s < start_s + 0.5 && std::abs(lp.lateral) < 0.3 &&
                         std::abs(lp.heading) < 30.0 * M_PI / 180.0;
    if (on_line && lp.s < start_s - 0.05) {
      if (!backOut(attempt)) {
        if (cancelled()) return cancelledResult();
        continue;
      }
    } else if (!on_line) {
      feedback(NAV_TO_START, attempt, "Navigating to the start point");
      setDockZone(false);
      if (!navigateToStart(start, cancelled)) {
        if (cancelled()) return cancelledResult();
        RCLCPP_WARN(node_.get_logger(), "Line docking: start point not reached");
        sleepFor(5.0, cancelled);
        continue;
      }
    }

    feedback(WAIT_GPS, attempt, "Waiting for RTK fixed at the start point");
    if (!waitGps(p_.gps_wait, cancelled)) {
      if (cancelled()) return cancelledResult();
      res.code = 903;
      res.message = "No RTK fixed at the start point";
      return res;
    }

    // Forward on the line to the gate; the last gps_window must prove the pose.
    feedback(APPROACH, attempt, "Following the dock line");
    if (!linePose(lp)) continue;
    rclcpp::Time window_start{0, 0, RCL_ROS_TIME};
    std::vector<double> yaws;
    auto d = driveTo(gate, p_.speed, Steer::LINE, false, (lp.s - gate) / p_.speed * 2.0 + 20.0, cancelled,
                     [&](const LinePose & now) {
                       if (now.s > gate + p_.gps_window) return;
                       if (window_start.nanoseconds() == 0) window_start = node_.now();
                       yaws.push_back(now.yaw);
                     });
    if (d != Drive::REACHED) {
      if (d == Drive::CANCELLED) return cancelledResult();
      RCLCPP_WARN(node_.get_logger(), "Line docking: approach ended: %s", driveText(static_cast<int>(d)));
      continue;
    }
    if (window_start.nanoseconds() == 0) window_start = node_.now();
    if (!sleepFor(0.3, cancelled)) return cancelledResult();
    std::string why;
    if (!linePose(lp) || !gateCheck(lp, window_start, yaws, why)) {
      RCLCPP_WARN(node_.get_logger(), "Line docking: pose not proven at the gate (%s): backing out",
                  why.empty() ? "no pose" : why.c_str());
      continue;
    }

    // Blind push: GPS off, heading on the line, no sideways correction.
    setDockZone(true);
    d = driveTo(-p_.push_overshoot, p_.push_speed, Steer::HOLD_HEADING, true,
                (gate + p_.push_overshoot) / p_.push_speed * 2.0 + 10.0, cancelled);
    if (d == Drive::CANCELLED) return cancelledResult();
    if (d != Drive::CONTACT) {
      RCLCPP_WARN(node_.get_logger(), "Line docking: no contact (%s): backing out", driveText(static_cast<int>(d)));
      continue;
    }
    feedback(WAIT_CHARGE, attempt, "Waiting for charge current");
    if (waitCharge(p_.charge_wait, cancelled)) {
      res.success = true;
      res.message = "Docked and charging";
      RCLCPP_INFO(node_.get_logger(), "Line docking: charging (attempt %d)", attempt);
      return res;
    }
    if (cancelled()) return cancelledResult();

    // Once more: back off a little and push again (a new contact for the station).
    RCLCPP_INFO(node_.get_logger(), "Line docking: no charge current after %.0f s: back off %.0f cm, push once more",
                p_.charge_wait, p_.backoff * 100.0);
    if (!linePose(lp)) continue;
    d = driveTo(lp.s + p_.backoff, 0.05, Steer::HOLD_HEADING, false, 3.0, cancelled);
    if (d == Drive::CANCELLED) return cancelledResult();
    if (!sleepFor(0.5, cancelled)) return cancelledResult();
    feedback(APPROACH, attempt, "Pushing once more");
    d = driveTo(-p_.push_overshoot, p_.push_speed, Steer::HOLD_HEADING, true, 10.0, cancelled);
    if (d == Drive::CANCELLED) return cancelledResult();
    if (d == Drive::CONTACT) {
      feedback(WAIT_CHARGE, attempt, "Waiting for charge current");
      if (waitCharge(p_.charge_wait, cancelled)) {
        res.success = true;
        res.message = "Docked and charging";
        RCLCPP_INFO(node_.get_logger(), "Line docking: charging after the second push (attempt %d)", attempt);
        return res;
      }
      if (cancelled()) return cancelledResult();
    }
    RCLCPP_WARN(node_.get_logger(), "Line docking: no charge current: backing out");
  }

  // Given up: don't leave the robot half in the dock.
  LinePose lp;
  if (linePose(lp) && lp.s < start_s - 0.05 && lp.s > -0.3 && std::abs(lp.lateral) < 0.3) backOut(p_.attempts);
  res.code = 906;
  res.message = "No charge after " + std::to_string(p_.attempts) + " attempts";
  return res;
}

}  // namespace open_mower_next::docking_helper

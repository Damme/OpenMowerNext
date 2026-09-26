#include "mower_logic/context.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>

namespace open_mower_next::mower_logic
{

namespace
{
bool insidePolygon(double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  bool c = false;
  const auto & p = poly.points;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    if ((p[i].y > y) != (p[j].y > y) && x < (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x) c = !c;
  }
  return c;
}

double distanceToEdges(double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  double best = 1e9;
  const auto & p = poly.points;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    const double dx = p[i].x - p[j].x, dy = p[i].y - p[j].y;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 > 0 ? std::clamp(((x - p[j].x) * dx + (y - p[j].y) * dy) / l2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(x - p[j].x - t * dx, y - p[j].y - t * dy));
  }
  return best;
}
}  // namespace

Context::Context(rclcpp::Node::SharedPtr n, Params p) : node(std::move(n)), params(std::move(p))
{
  tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf);
  coverage_client = node->create_client<open_mower_next::srv::AreaCoverage>("/area_coverage");
  emergency_client_ = node->create_client<std_srvs::srv::SetBool>("/worx/emergency");
  blade_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/mower_controller/commands", 10);
  drive_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10);

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/power", 10, [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      battery_ = m->percentage;
    });
  charger_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/power/charger_present", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      charger_ = m->data;
    });
  worx_sub_ = node->create_subscription<open_mower_next::msg::WorxStatus>(
    "/worx/status", 10, [this](open_mower_next::msg::WorxStatus::ConstSharedPtr m) {
      bool bumped = false;
      {
        std::lock_guard<std::mutex> l(mutex_);
        bumped = bumps_seen_ && m->bumps != *bumps_seen_;
        bumps_seen_ = m->bumps;
      }
      if (bumped) onBump();
      std::lock_guard<std::mutex> l(mutex_);
      emergency_ = m->emergency || m->board_emergency == 1;
      worx_emergency_ = m->emergency;
      board_emergency_ = m->board_emergency == 1;
      lift_ = m->lift;
      collision_ = m->collision;
      for (size_t i = 0; i < m->digital_names.size() && i < m->digital_active.size(); ++i) {
        if (m->digital_names[i] == "Rain" && m->digital_active[i]) last_rain_ = Clock::now();
      }
    });
  if (params.require_gps) {
    gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      params.gps_fix_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        const double acc = std::sqrt(std::max(m->position_covariance[0], m->position_covariance[4]));
        const bool good = m->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
                          m->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
                          acc <= params.gps_max_accuracy;
        std::lock_guard<std::mutex> l(mutex_);
        if (good) {
          if (!gps_good_since_) gps_good_since_ = Clock::now();
          gps_last_good_ = Clock::now();
        } else {
          gps_good_since_.reset();
        }
      });
  }
  // Sensor-data QoS like the costmap obstacle layer; republished so late
  // subscribers (and a cleared costmap) get the points again.
  obstacle_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/bump_obstacles", rclcpp::SensorDataQoS());
  obstacle_timer_ = node->create_wall_timer(std::chrono::seconds(1), [this]() { publishObstacles(); });
  map_sub_ = node->create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(1).transient_local().reliable(),
    [this](open_mower_next::msg::Map::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      map_ = *m;
    });
}

double Context::batteryFraction() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return battery_;
}

bool Context::charging() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return charger_;
}

bool Context::atDock() const
{
  if (charging()) return true;
  const auto pose = robotPose();
  std::lock_guard<std::mutex> l(mutex_);
  if (!pose) return false;
  for (const auto & d : map_.docking_stations) {
    const auto & p = d.pose.pose.position;
    if (std::hypot(p.x - pose->pose.position.x, p.y - pose->pose.position.y) < params.undock_distance) return true;
  }
  return false;
}

bool Context::emergency() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return emergency_;
}

bool Context::clearEmergency(std::string & message)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (lift_) {
      message = "refused: the robot is lifted (Lift input active)";
      return false;
    }
    if (collision_) {
      message = "refused: bumper still pressed";
      return false;
    }
    if (board_emergency_) {
      // Tilt with the blade on: the firmware cut the motor MOSFET (MOTORREQ_EMGSTOP) and
      // only accepts ping/ENABLE/DISABLE/SETSPEED over SPI, so it can't be reset from here.
      message = "refused: firmware emergency (tilt) - needs MOTORREQ_RESETEMG support in the firmware or a board reset";
      return false;
    }
    if (!worx_emergency_) {
      message = "no emergency latched";
      return true;
    }
  }
  if (!emergency_client_->service_is_ready()) {
    message = "/worx/emergency not available";
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = false;
  emergency_client_->async_send_request(req);
  message = "emergency cleared";
  return true;
}

bool Context::raining()
{
  if (!params.dock_on_rain) return false;
  std::lock_guard<std::mutex> l(mutex_);
  return last_rain_ && std::chrono::duration<double>(Clock::now() - *last_rain_).count() < params.rain_clear_delay;
}

bool Context::gpsOk()
{
  if (!params.require_gps) return true;
  std::lock_guard<std::mutex> l(mutex_);
  const auto now = Clock::now();
  if (!gps_last_good_ || std::chrono::duration<double>(now - *gps_last_good_).count() > params.gps_timeout) {
    gps_good_since_.reset();
    return false;
  }
  return gps_good_since_ && std::chrono::duration<double>(now - *gps_good_since_).count() >= params.gps_settle;
}

bool Context::needsCharging()
{
  std::lock_guard<std::mutex> l(mutex_);
  if (std::isnan(battery_)) return needs_charging_;
  if (battery_ < params.battery_low) needs_charging_ = true;
  if (battery_ >= params.battery_resume) needs_charging_ = false;
  return needs_charging_;
}

std::vector<std::string> Context::operationAreas() const
{
  std::lock_guard<std::mutex> l(mutex_);
  std::vector<std::string> ids;
  for (const auto & a : map_.areas) {
    if (a.type != open_mower_next::msg::Area::TYPE_OPERATION) continue;
    if (params.areas.empty() || std::find(params.areas.begin(), params.areas.end(), a.id) != params.areas.end()) {
      ids.push_back(a.id);
    }
  }
  return ids;
}

std::optional<geometry_msgs::msg::PoseStamped> Context::robotPose() const
{
  try {
    const auto t = tf->lookupTransform("map", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped p;
    p.header = t.header;
    p.pose.position.x = t.transform.translation.x;
    p.pose.position.y = t.transform.translation.y;
    p.pose.orientation = t.transform.rotation;
    return p;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void Context::drive(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "base_link";
  t.twist.linear.x = linear;
  t.twist.angular.z = angular;
  drive_pub_->publish(t);
}

void Context::setBlade(bool on)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (on != blade_on_) {
      RCLCPP_INFO(node->get_logger(), "Blade %s", on ? "ON" : "OFF");
    }
    blade_on_ = on;
  }
  std_msgs::msg::Float64MultiArray m;
  m.data = {on ? 1.0 : 0.0};
  blade_pub_->publish(m);
}

std::string Context::lastBranch() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return branch_;
}

void Context::setBranch(const std::string & b)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (b != branch_) RCLCPP_INFO(node->get_logger(), "State: %s", b.c_str());
  branch_ = b;
}

void Context::onBump()
{
  const auto pose = robotPose();
  if (!pose) {
    RCLCPP_WARN(node->get_logger(), "Bump, but no robot pose - obstacle not marked");
    std::lock_guard<std::mutex> l(mutex_);
    last_bump_ = Bump{Clock::now(), NAN, NAN};
    return;
  }
  const auto & q = pose->pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  Bump b;
  b.time = Clock::now();
  b.x = pose->pose.position.x + params.bump_front_offset * std::cos(yaw);
  b.y = pose->pose.position.y + params.bump_front_offset * std::sin(yaw);
  b.yaw = yaw;
  RCLCPP_WARN(node->get_logger(), "Bump: obstacle marked at (%.2f, %.2f)", b.x, b.y);
  {
    std::lock_guard<std::mutex> l(mutex_);
    last_bump_ = b;
    // Repeated bumps at the same obstacle refresh it instead of stacking strips
    // (four stacked strips closed a narrow corridor in the sim).
    auto same = std::find_if(obstacles_.begin(), obstacles_.end(), [&](const Bump & o) {
      return std::hypot(o.x - b.x, o.y - b.y) < params.bump_merge_distance;
    });
    if (same != obstacles_.end()) {
      *same = b;
    } else {
      obstacles_.push_back(b);
    }
  }
  publishObstacles();
}

std::optional<Context::Bump> Context::knownObstacleNear(double x, double y, double radius) const
{
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & o : obstacles_) {
    if (std::hypot(o.x - x, o.y - y) <= radius) return o;
  }
  return std::nullopt;
}

std::optional<Context::Bump> Context::lastBump() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return last_bump_;
}

bool Context::bumpedSince(Clock::time_point t) const
{
  std::lock_guard<std::mutex> l(mutex_);
  return last_bump_ && last_bump_->time > t;
}

std::optional<Context::Bump> Context::takeBump()
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!last_bump_ || last_bump_->time <= handled_bump_) return std::nullopt;
  handled_bump_ = last_bump_->time;
  return last_bump_;
}

void Context::clearBumpObstacles()
{
  std::lock_guard<std::mutex> l(mutex_);
  obstacles_.clear();
  if (last_bump_) handled_bump_ = last_bump_->time;
}

void Context::publishObstacles()
{
  std::vector<std::pair<double, double>> pts;
  const auto robot = robotPose();
  double rc = 1.0, rs = 0.0;
  if (robot) {
    const auto & q = robot->pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    rc = std::cos(yaw);
    rs = std::sin(yaw);
  }
  // Inside the robot's current footprint grown by bump_keep_free?
  auto near_robot = [&](double px, double py) {
    if (!robot) return false;
    const double dx = px - robot->pose.position.x, dy = py - robot->pose.position.y;
    const double u = dx * rc + dy * rs, v = -dx * rs + dy * rc;
    const double m = params.bump_keep_free;
    return u > -params.footprint_rear - m && u < params.footprint_front + m &&
           std::abs(v) < params.footprint_half_width + m;
  };
  {
    std::lock_guard<std::mutex> l(mutex_);
    // Per bump: discs at the front-left corner, centre and front-right corner
    // (5 cm spacing, the costmap resolution). Points outside every area are
    // useless (already blocked) and dropped.
    const double r = params.bump_obstacle_radius, hw = params.footprint_half_width;
    auto inside_areas = [&](double px, double py) {
      bool in = false;
      for (const auto & a : map_.areas) {
        if (a.area.polygon.points.size() < 3 || !insidePolygon(px, py, a.area.polygon)) continue;
        if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION) return false;
        in = true;
      }
      return in;
    };
    for (const auto & o : obstacles_) {
      // o.x/o.y is the centre disc; the corner discs sit hw to either side.
      const double c = std::cos(o.yaw), s = std::sin(o.yaw);
      for (double side : {-hw, 0.0, hw}) {
        const double cx = o.x - side * s, cy = o.y + side * c;
        for (double dx = -r; dx <= r + 1e-9; dx += 0.05) {
          for (double dy = -r; dy <= r + 1e-9; dy += 0.05) {
            const double px = cx + dx, py = cy + dy;
            if (dx * dx + dy * dy <= r * r && !near_robot(px, py) && inside_areas(px, py)) pts.emplace_back(px, py);
          }
        }
      }
    }
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & [px, py] : pts) {
    *x = static_cast<float>(px);
    *y = static_cast<float>(py);
    *z = 0.1f;
    ++x, ++y, ++z;
  }
  obstacle_pub_->publish(cloud);
}


std::optional<geometry_msgs::msg::PoseStamped> Context::transitVia(const geometry_msgs::msg::PoseStamped & goal)
{
  if (params.transit_jitter <= 0.0) return std::nullopt;
  const auto start = robotPose();
  if (!start) return std::nullopt;
  const double sx = start->pose.position.x, sy = start->pose.position.y;
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double len = std::hypot(gx - sx, gy - sy);
  if (len < params.transit_jitter_min_distance) return std::nullopt;
  const double nx = -(gy - sy) / len, ny = (gx - sx) / len;  // left normal
  std::lock_guard<std::mutex> l(mutex_);
  // Point inside a non-exclusion area and >= margin from every area edge.
  auto open_lawn = [&](double x, double y) {
    bool in_area = false;
    for (const auto & a : map_.areas) {
      if (a.area.polygon.points.size() < 3) continue;
      const bool inside = insidePolygon(x, y, a.area.polygon);
      if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION && inside) return false;
      if (a.type != open_mower_next::msg::Area::TYPE_EXCLUSION && inside) in_area = true;
      if (distanceToEdges(x, y, a.area.polygon) < params.transit_via_margin) return false;
    }
    return in_area;
  };
  // Only vary transits across open lawn; in corridors a via point makes an S-bend.
  const int steps = static_cast<int>(len / 0.5);
  for (int i = 1; i < steps; ++i) {
    const double u = static_cast<double>(i) / steps;
    if (!open_lawn(sx + u * (gx - sx), sy + u * (gy - sy))) return std::nullopt;
  }
  std::uniform_real_distribution<double> along(0.35, 0.65), side(-params.transit_jitter, params.transit_jitter);
  for (int attempt = 0; attempt < 8; ++attempt) {
    const double u = along(rng_), o = side(rng_);
    const double x = sx + u * (gx - sx) + o * nx, y = sy + u * (gy - sy) + o * ny;
    if (!open_lawn(x, y)) continue;
    geometry_msgs::msg::PoseStamped via = goal;
    via.pose.position.x = x;
    via.pose.position.y = y;
    const double yaw = std::atan2(gy - sy, gx - sx);  // heading along the transit
    via.pose.orientation.z = std::sin(yaw / 2);
    via.pose.orientation.w = std::cos(yaw / 2);
    return via;
  }
  return std::nullopt;
}

}  // namespace open_mower_next::mower_logic

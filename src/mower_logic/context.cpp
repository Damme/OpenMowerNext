#include "mower_logic/context.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>

namespace open_mower_next::mower_logic
{

Context::Context(rclcpp::Node::SharedPtr n, Params p) : node(std::move(n)), params(std::move(p))
{
  tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf);
  coverage_client = node->create_client<open_mower_next::srv::AreaCoverage>("/area_coverage");
  blade_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/mower_controller/commands", 10);

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

bool Context::emergency() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return emergency_;
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
  RCLCPP_WARN(node->get_logger(), "Bump: obstacle marked at (%.2f, %.2f)", b.x, b.y);
  {
    std::lock_guard<std::mutex> l(mutex_);
    last_bump_ = b;
    obstacles_.emplace_back(b.x, b.y);
  }
  publishObstacles();
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
  {
    std::lock_guard<std::mutex> l(mutex_);
    // A disc of points per obstacle (5 cm spacing, the costmap resolution).
    const double r = params.bump_obstacle_radius;
    for (const auto & [cx, cy] : obstacles_) {
      for (double dx = -r; dx <= r + 1e-9; dx += 0.05) {
        for (double dy = -r; dy <= r + 1e-9; dy += 0.05) {
          if (dx * dx + dy * dy <= r * r) pts.emplace_back(cx + dx, cy + dy);
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

}  // namespace open_mower_next::mower_logic

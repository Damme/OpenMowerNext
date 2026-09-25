#include "mower_logic/context.hpp"

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

}  // namespace open_mower_next::mower_logic

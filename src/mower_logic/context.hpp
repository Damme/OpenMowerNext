#pragma once
// Shared state of the mower_logic executor: robot inputs (battery, charger,
// GPS, emergency, rain), the operator command, mission progress and the ROS
// handles the behaviour tree nodes use.

#include "mower_logic/mission.hpp"

#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/msg/worx_status.hpp"
#include "open_mower_next/srv/area_coverage.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>

namespace open_mower_next::mower_logic
{

enum class Command { IDLE, MOW, HOME };

inline const char * toString(Command c)
{
  switch (c) {
    case Command::MOW: return "MOW";
    case Command::HOME: return "HOME";
    default: return "IDLE";
  }
}

struct Params
{
  double battery_low = 0.20;         // go charging below this fraction
  double battery_resume = 0.95;      // resume mowing at this fraction
  bool require_gps = true;
  std::string gps_fix_topic = "/gps/fix";
  double gps_max_accuracy = 0.10;    // m (sqrt of horizontal covariance)
  double gps_timeout = 1.5;          // s without a good fix -> pause
  double gps_settle = 10.0;          // s of good fixes before continuing (OM_GPS_WAIT_TIME_SEC)
  bool dock_on_rain = true;
  double rain_clear_delay = 1800.0;  // s without rain before mowing again
  int max_pass_attempts = 3;
  int max_dock_attempts = 3;
  double blade_spinup = 2.0;         // s between blade on and driving
  double resume_backtrack = 0.5;     // m re-mowed before a resume point
  std::string dock_type = "openmower";
  std::string controller_id = "FollowPath";
  std::string goal_checker_id = "general_goal_checker";
  std::string progress_checker_id = "";  // empty = controller_server default
  std::vector<std::string> areas;    // operation area ids to mow; empty = all, in map order
};

class Context
{
public:
  Context(rclcpp::Node::SharedPtr node, Params params);

  rclcpp::Node::SharedPtr node;
  Params params;
  Mission mission;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::atomic<Command> command{Command::IDLE};
  std::atomic<int> dock_failures{0};
  std::atomic<bool> blade_in_use{false};  // set by FollowPass while it runs
  std::atomic<bool> count_failure{true};  // false: last pass "failure" was an early end, resume without counting

  // ---- inputs ----
  double batteryFraction() const;
  bool charging() const;   // charger present
  bool emergency() const;
  bool raining();          // latched for rain_clear_delay after the last rain
  bool gpsOk();            // good fixes for gps_settle, none missing for gps_timeout
  bool needsCharging();    // latched: set below battery_low, cleared at battery_resume
  std::vector<std::string> operationAreas() const;
  std::optional<geometry_msgs::msg::PoseStamped> robotPose() const;

  void setBlade(bool on);
  rclcpp::Client<open_mower_next::srv::AreaCoverage>::SharedPtr coverage_client;

  std::string lastBranch() const;
  void setBranch(const std::string & b);

private:
  using Clock = std::chrono::steady_clock;
  mutable std::mutex mutex_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double battery_ = NAN;
  bool charger_ = false;
  bool emergency_ = false;
  std::optional<Clock::time_point> last_rain_;
  std::optional<Clock::time_point> gps_good_since_, gps_last_good_;
  bool needs_charging_ = false;
  open_mower_next::msg::Map map_;
  std::string branch_;
  bool blade_on_ = false;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
  rclcpp::Subscription<open_mower_next::msg::WorxStatus>::SharedPtr worx_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr blade_pub_;
};

}  // namespace open_mower_next::mower_logic

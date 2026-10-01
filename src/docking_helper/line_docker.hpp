#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>

#include "open_mower_next/msg/area.hpp"
#include "open_mower_next/msg/worx_status.hpp"

namespace open_mower_next::docking_helper
{

// Docking along the dock's line (Daniel 2026-10-01), replacing Nav2's docking
// server, whose control law steered to a point and turned in place in the dock
// slot short of the contacts (log 2026-09-30: 3 x 30 s timeouts, one charge start
// cut off by the timeout).
//
// One attempt:
//  1. Start point on the line, approach_length before the gate (Nav2 navigate_to_pose
//     the first time, otherwise straight back along the line).
//  2. Follow the line forward with GPS to the gate, just inside the mow area where
//     the line leaves it (the dock and its roof are outside).
//  3. Gate: the pose must be proven - RTK all the way through the last gps_window,
//     the GPS track's direction = the EKF heading, EKF = GPS position, and on the
//     line. Anything off: back out and try again rather than steer in the slot.
//  4. Blind push: GPS off, heading held on the line, no sideways correction. Stops on
//     the bumper (BlockForward), a charge start (powerState), a stall or the charger.
//  5. Wait charge_wait for charge current; none: back off a little, push once more,
//     wait again; none: back out to the start point (next attempt).
class LineDocker
{
public:
  struct Params
  {
    double approach_length = 3.0;   // m: start point before the gate
    double gate_margin = 0.1;       // m: gate this far inside the mow area
    double gate_fallback = 1.0;     // m from the dock pose, when the map gives no edge
    double speed = 0.15;            // m/s on the line
    double push_speed = 0.10;       // m/s blind push
    double reverse_speed = 0.15;    // m/s back out
    double k_lateral = 2.5;         // heading target = line - atan(k_lateral * offset)
    double k_heading = 1.5;         // 1/s
    double max_angular = 0.5;       // rad/s
    double lateral_tolerance = 0.05;          // m at the gate
    double heading_tolerance = 4.0 * M_PI / 180.0;
    bool require_gps = true;
    double gps_window = 1.5;                  // m before the gate: RTK all the way, track = heading
    double gps_max_gap = 1.0;                 // s between GPS fixes inside the window
    double gps_heading_tolerance = 3.0 * M_PI / 180.0;
    double gps_position_tolerance = 0.05;     // m, EKF vs GPS at the same stamp
    double gps_wait = 120.0;                  // s for RTK at the start point
    double push_overshoot = 0.15;             // m past the dock pose without contact = missed
    double backoff = 0.03;                    // m before the second push
    double charge_wait = 5.0;                 // s for charge current after contact
    int attempts = 5;
    double nav_timeout = 300.0;               // s to the start point
  };

  // Feedback status codes (= DockRobotNearest/To feedback STATUS_*).
  enum Status : uint16_t
  {
    NAV_TO_START = 1,
    WAIT_GPS = 2,
    APPROACH = 3,
    WAIT_CHARGE = 4,
    BACK_OUT = 5,
  };

  struct Result
  {
    bool success = false;
    uint16_t code = 0;  // DockRobotNearest result codes
    std::string message;
    int attempts = 0;
  };

  using FeedbackFn = std::function<void(uint16_t status, int attempt, const std::string & message)>;

  // dock_zone(true) on entering the dock zone (between gate and dock): the owner
  // gates GPS off and switches worx_hardware's docking mode on; false on leaving.
  LineDocker(rclcpp::Node & node, std::shared_ptr<tf2_ros::Buffer> tf, std::function<void(bool)> dock_zone);

  // Blocking. dock_pose: base_link on the charger (map frame).
  Result dock(const geometry_msgs::msg::Pose & dock_pose, const std::vector<open_mower_next::msg::Area> & areas,
              const FeedbackFn & feedback, const std::function<bool()> & cancelled);

  // Distance from the dock pose (along the line, outwards) where the line leaves
  // the mow/navigation areas; NAN if it doesn't within max_s.
  static double areaEdge(const geometry_msgs::msg::Pose & dock_pose,
                         const std::vector<open_mower_next::msg::Area> & areas, double max_s);

private:
  struct LinePose
  {
    double s = 0.0;        // m out from the dock pose along the line
    double lateral = 0.0;  // m, left of the line (facing the dock) positive
    double heading = 0.0;  // rad, yaw - line yaw
    double yaw = 0.0;
    rclcpp::Time stamp;
  };
  struct GpsSample
  {
    rclcpp::Time stamp;
    double x, y;
  };
  enum class Drive { REACHED, CONTACT, COLLISION, OVERSHOOT, TIMEOUT, NO_POSE, CANCELLED };
  enum class Steer { LINE, HOLD_HEADING };

  bool linePose(LinePose & lp) const;
  void publish(double v, double w);
  void stop();
  // Drive along the line to s = target (forward when target < current s).
  Drive driveTo(double target, double speed, Steer steer, bool stop_on_contact, double timeout,
                const std::function<bool()> & cancelled, const std::function<void(const LinePose &)> & on_tick = {});
  bool gateCheck(const LinePose & at_gate, const rclcpp::Time & window_start, const std::vector<double> & yaws,
                 std::string & why);
  bool waitGps(double timeout, const std::function<bool()> & cancelled);
  bool waitCharge(double timeout, const std::function<bool()> & cancelled);
  bool navigateToStart(const geometry_msgs::msg::Pose & start, const std::function<bool()> & cancelled);
  bool sleepFor(double seconds, const std::function<bool()> & cancelled);
  void setDockZone(bool on);
  bool charger() const;

  rclcpp::Node & node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::function<void(bool)> dock_zone_;
  bool in_dock_zone_ = false;
  Params p_;

  // Line of the current dock (map frame): dock pose and its yaw.
  double dock_x_ = 0.0, dock_y_ = 0.0, dock_yaw_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Subscription<open_mower_next::msg::WorxStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  mutable std::mutex mutex_;
  std::deque<GpsSample> gps_;  // /odometry/gps: only RTK fixed passes docking_helper's gate
  open_mower_next::msg::WorxStatus::ConstSharedPtr status_;
  rclcpp::Time status_time_{0, 0, RCL_ROS_TIME};
  bool charger_ = false;
};

}  // namespace open_mower_next::docking_helper

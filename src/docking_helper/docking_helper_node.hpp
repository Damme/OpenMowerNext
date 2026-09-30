#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/msg/docking_station.hpp"
#include "open_mower_next/srv/find_nearest_docking_station.hpp"
#include "open_mower_next/action/dock_robot_nearest.hpp"
#include "open_mower_next/action/dock_robot_to.hpp"

namespace open_mower_next::docking_helper
{
class DockingHelperNode : public rclcpp::Node
{
public:
  explicit DockingHelperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DockingHelperNode();

private:
  // Action server types
  using DockRobotNearestAction = open_mower_next::action::DockRobotNearest;
  using DockRobotNearestGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotNearestAction>;

  using DockRobotToAction = open_mower_next::action::DockRobotTo;
  using DockRobotToGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotToAction>;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);
  std::shared_ptr<rclcpp::Subscription<open_mower_next::msg::Map>> map_sub_;
  std::mutex docking_stations_mutex_;
  std::vector<open_mower_next::msg::DockingStation> docking_stations_;  // guarded by docking_stations_mutex_

  std::shared_ptr<open_mower_next::msg::DockingStation> findNearestDockingStation();
  void findNearestDockingStationService(
      const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
      std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response);

  std::shared_ptr<geometry_msgs::msg::PoseStamped>
  dockPose(const std::shared_ptr<open_mower_next::msg::DockingStation>& station);

  rclcpp::Service<open_mower_next::srv::FindNearestDockingStation>::SharedPtr find_nearest_docking_station_service_;

  rclcpp_action::Client<nav2_msgs::action::DockRobot>::SharedPtr dock_client_;
  // worx_hardware's docking mode (BlockForward at the dock is no bump) for the final approach.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr docking_mode_client_;
  void setDockingMode(bool on);

  rclcpp_action::Server<DockRobotNearestAction>::SharedPtr dock_robot_nearest_server_;
  rclcpp_action::Server<DockRobotToAction>::SharedPtr dock_robot_to_server_;

  rclcpp_action::GoalResponse handleDockRobotNearestGoal(const rclcpp_action::GoalUUID& uuid,
                                                         std::shared_ptr<const DockRobotNearestAction::Goal> goal);
  rclcpp_action::CancelResponse
  handleDockRobotNearestCancel(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);
  void handleDockRobotNearestAccepted(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handleDockRobotToGoal(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const DockRobotToAction::Goal> goal);
  rclcpp_action::CancelResponse handleDockRobotToCancel(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);
  void handleDockRobotToAccepted(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);

  std::shared_ptr<open_mower_next::msg::DockingStation> findDockingStationById(const std::string& id);

  // On the charger the robot stands at its docking station's recorded pose:
  // localization (ekf_se_map) is set to it once per stay (holdRestPose keeps it there).
  void setPoseWhenDocked();
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::TimerBase::SharedPtr set_pose_timer_;
  std::atomic<bool> charger_present_{false};
  std::atomic<bool> docked_anew_{false};  // charger came back: log the next set
  rclcpp::Time last_docked_pose_{0, 0, RCL_ROS_TIME};  // timer only
  // Wheel odometry: no pose reset while the wheels turn or just did (the charge
  // current outlasts the start of an undock by ~2 s).
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::atomic<int64_t> last_motion_ns_{0};
  std::atomic<bool> charger_known_{false};  // a /power/charger_present message arrived

  // Pose across restarts (pose_file): ekf_se_map starts at x=y=yaw=0, and off the
  // dock nothing but a drive with RTK gives the heading back. The map pose is saved
  // once the wheels have stood still for a while (stop, idle, pause, docked) - the
  // only write. The file may be stale (stack stopped while driving): at startup it is
  // restored (not on the charger: the dock pose wins) only if RTK fixed puts the robot
  // at the saved position, or unchecked after pose_restore_gps_wait without RTK.
  void persistPose();
  void savePose();
  void restorePose();
  std::string pose_file_;
  double pose_restore_gps_wait_ = 60.0;  // s
  double pose_restore_max_offset_ = 0.5;  // m, saved position vs GPS
  bool pose_saved_ = false;               // timer only: the file holds the current rest pose
  bool pose_restore_done_ = false;        // timer only
  rclcpp::Time start_time_;
  std::atomic<int64_t> first_gps_ns_{0};  // first fix the GPS gate let through
  bool set_pose_when_docked_ = false;

  // Rest pose: taken once when the robot comes to rest (dock pose on the charger, the
  // restored or the current pose elsewhere) and never re-taken while it stands.
  // The gyro bias still turns the EKF heading while standing (~3 deg/h; 45 deg in a
  // 14 h night on the dock 2026-09-30 reversed the undock off the approach line), so
  // localization is set back to it when it drifts more than rest_hold_yaw (or, on the
  // charger, rest_hold_dist). Replaces the dock set every 5 s (log spam).
  void setRestPose(double x, double y, double yaw);
  void holdRestPose();
  bool rest_valid_ = false;  // timer only
  int64_t last_set_ns_ = 0;  // timer only: last set_pose sent
  double rest_x_ = 0.0, rest_y_ = 0.0, rest_yaw_ = 0.0;
  double rest_hold_yaw_ = 1.0 * M_PI / 180.0;  // rad
  double rest_hold_dist_ = 0.05;               // m

  // GPS gate for localization: fixes pass only off the charger (and RTK fixed).
  void gateGps(sensor_msgs::msg::NavSatFix::ConstSharedPtr m);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  bool gps_require_rtk_fixed_ = true;
  // Final docking approach (at the staging pose and in, incl. retries): no GPS either.
  // Under the dock roof RTK flips fixed/float with false fixes; 2026-09-29 three
  // approaches ended 10-20 cm beside the contacts while GPS kept switching on/off.
  std::atomic<bool> final_approach_{false};
  std::string gps_gate_reason_ = "starting";

  template <typename ActionT, typename GoalHandleT>
  void executeDockingAction(const std::shared_ptr<GoalHandleT>& goal_handle,
                            const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station);
};

}  // namespace open_mower_next::docking_helper

#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
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
  // localization (ekf_se_map) is set to it, heading included, once per docking.
  void setPoseWhenDocked();
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::TimerBase::SharedPtr set_pose_timer_;
  std::atomic<bool> charger_present_{false};
  std::atomic<bool> docked_pose_set_{false};

  // GPS gate for localization: fixes pass only off the charger (and RTK fixed).
  void gateGps(sensor_msgs::msg::NavSatFix::ConstSharedPtr m);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  bool gps_require_rtk_fixed_ = true;
  std::string gps_gate_reason_ = "starting";

  template <typename ActionT, typename GoalHandleT>
  void executeDockingAction(const std::shared_ptr<GoalHandleT>& goal_handle,
                            const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station);
};

}  // namespace open_mower_next::docking_helper

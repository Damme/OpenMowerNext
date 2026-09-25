#pragma once
// FTC ("follow the carrot") path controller for Nav2, ported from the
// ftc_local_planner (MBF) of the Worx ROS1 stack, including its turn assist,
// stuck recovery, carrot leash and output slew limit.
//
// A carrot moves along the plan at speed_fast/speed_slow (and speed_angular);
// PID on the carrot's longitudinal/lateral/angular error in base_link gives
// the command. States: PRE_ROTATE -> FOLLOWING -> WAITING_FOR_GOAL_APPROACH ->
// POST_ROTATE -> FINISHED. Completion means the carrot traversed the whole plan
// (use FTCGoalChecker): unlike a proximity goal checker, closed loops work.

#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Geometry>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace open_mower_next::ftc_controller
{

// Oscillation detector (TEB's FailureDetector): sign changes of the angular
// command while the mean command stays small.
class FailureDetector
{
public:
  void setBufferLength(size_t n) { capacity_ = n; while (buf_.size() > capacity_) buf_.pop_front(); }
  void update(double v, double omega, double v_max, double omega_max, double v_eps, double omega_eps);
  bool isOscillating() const { return oscillating_; }
  void clear() { buf_.clear(); oscillating_ = false; }

private:
  struct M { double v, omega; };
  std::deque<M> buf_;
  size_t capacity_ = 50;
  bool oscillating_ = false;
};

struct FtcConfig
{
  double speed_fast = 0.5, speed_fast_threshold = 1.5, speed_fast_threshold_angle = 5.0;
  double speed_slow = 0.2, speed_angular = 20.0, acceleration = 1.0;
  double lateral_priority_distance = 0.0;
  double kp_lon = 1.0, ki_lon = 0.0, ki_lon_max = 10.0, kd_lon = 0.0;
  double kp_lat = 1.0, ki_lat = 0.0, ki_lat_max = 10.0, kd_lat = 0.0;
  double kp_ang = 1.0, ki_ang = 0.0, ki_ang_max = 10.0, kd_ang = 0.0;
  double max_cmd_vel_speed = 2.0, max_cmd_vel_ang = 2.0;
  double max_goal_distance_error = 1.0, max_goal_angle_error = 10.0, goal_timeout = 5.0;
  double max_follow_distance = 1.0;
  bool forward_only = true;
  double carrot_max_lag = 0.40, max_cmd_vel_accel = 0.40;
  bool oscillation_recovery = true;
  double oscillation_v_eps = 5.0, oscillation_omega_eps = 5.0, oscillation_recovery_min_duration = 5.0;
  bool check_obstacles = true, obstacle_footprint = true;
  int obstacle_lookahead = 5;
};

class FTCController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                           const geometry_msgs::msg::Twist & velocity,
                                                           nav2_core::GoalChecker * goal_checker) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

  // Registry read by FTCGoalChecker (same controller_server process).
  static bool finished(const std::string & controller_name);

private:
  enum PlannerState { PRE_ROTATE, FOLLOWING, WAITING_FOR_GOAL_APPROACH, POST_ROTATE, FINISHED };

  double now() const;
  double timeInState() const { return now() - state_entered_time_; }
  void setState(PlannerState s);
  double distanceLookahead();
  PlannerState updatePlannerState();
  void updateControlPoint(double dt);
  void calculateVelocityCommands(double dt, geometry_msgs::msg::TwistStamped & cmd);
  bool turnAssist(bool oscillating, geometry_msgs::msg::TwistStamped & cmd);
  bool recoveryActive(const Eigen::Vector2d & p, geometry_msgs::msg::TwistStamped & cmd);
  void skipAhead(double dist);
  bool checkCollision(int max_points);
  bool checkOscillation(const geometry_msgs::msg::TwistStamped & cmd);
  void publishFinished(bool f);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FTCController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> plan_pub_;
  FtcConfig cfg_;
  double speed_limit_ = 0.0;  // absolute m/s, 0 = none

  PlannerState state_ = FINISHED;
  double state_entered_time_ = 0.0;
  bool is_crashed_ = false;
  std::vector<geometry_msgs::msg::PoseStamped> plan_;
  std::string plan_frame_ = "map";
  Eigen::Affine3d current_control_point_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d local_control_point_ = Eigen::Affine3d::Identity();

  double lat_error_ = 0, lon_error_ = 0, angle_error_ = 0;
  double last_lon_error_ = 0, last_lat_error_ = 0, last_angle_error_ = 0;
  double i_lon_error_ = 0, i_lat_error_ = 0, i_angle_error_ = 0;
  double last_cmd_vel_linear_ = 0;
  bool carrot_gated_ = false;
  double current_movement_speed_ = 0;
  uint32_t current_index_ = 0;
  double current_progress_ = 0;
  double last_time_ = 0;

  // stuck recovery
  int recovery_phase_ = 0;
  double recovery_phase_start_ = 0;
  Eigen::Vector2d stuck_ref_pos_{0, 0};
  double stuck_ref_time_ = 0;
  bool stuck_ref_valid_ = false;
  int recovery_attempts_ = 0;
  uint32_t recovery_last_index_ = 0;

  // turn assist
  Eigen::Vector2d robot_pos_{0, 0};
  bool turn_ref_valid_ = false;
  Eigen::Vector2d turn_start_pos_{0, 0};
  double turn_ref_error_ = 0, turn_ref_time_ = 0, turn_assist_until_ = 0;

  // oscillation
  FailureDetector failure_detector_;
  double time_last_oscillation_ = 0;
  bool oscillation_detected_ = false, oscillation_warning_ = false;
};

// Goal is reached when the named FTCController finished traversing the plan.
class FTCGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
                     const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  std::string controller_ = "FTC";
};

}  // namespace open_mower_next::ftc_controller

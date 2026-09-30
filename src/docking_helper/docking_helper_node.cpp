#include "docking_helper/docking_helper_node.hpp"
#include "docking_helper_node.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <functional>
#include <future>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::placeholders;

open_mower_next::docking_helper::DockingHelperNode::DockingHelperNode(const rclcpp::NodeOptions& options)
  : Node("docking_helper", options)
{
  map_sub_ = create_subscription<open_mower_next::msg::Map>(
      "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
      std::bind(&DockingHelperNode::mapCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  find_nearest_docking_station_service_ = create_service<open_mower_next::srv::FindNearestDockingStation>(
      "/find_nearest_docking_station", std::bind(&DockingHelperNode::findNearestDockingStationService, this,
                                                 std::placeholders::_1, std::placeholders::_2));

  dock_client_ = rclcpp_action::create_client<nav2_msgs::action::DockRobot>(this, "/dock_robot");
  docking_mode_client_ = create_client<std_srvs::srv::SetBool>("/worx/docking_mode");

  dock_robot_nearest_server_ = rclcpp_action::create_server<DockRobotNearestAction>(
      this, "dock_robot_nearest", std::bind(&DockingHelperNode::handleDockRobotNearestGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotNearestCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotNearestAccepted, this, _1));

  dock_robot_to_server_ = rclcpp_action::create_server<DockRobotToAction>(
      this, "dock_robot_to", std::bind(&DockingHelperNode::handleDockRobotToGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotToCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotToAccepted, this, _1));

  const bool set_pose = set_pose_when_docked_ = declare_parameter("set_pose_when_docked", false);
  const bool gps_gate = declare_parameter("gps_gate", false);
  pose_file_ = declare_parameter("pose_file", std::string(""));
  pose_restore_gps_wait_ = declare_parameter("pose_restore_gps_wait", pose_restore_gps_wait_);
  pose_restore_max_offset_ = declare_parameter("pose_restore_max_offset", pose_restore_max_offset_);
  start_time_ = now();
  if (set_pose || gps_gate || !pose_file_.empty())
  {
    charger_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/power/charger_present", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data && !charger_present_) docked_anew_ = true;
          charger_present_ = m->data;
          charger_known_ = true;
        });
  }
  if (set_pose || !pose_file_.empty())
  {
    set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        declare_parameter("set_pose_topic", std::string("/ekf_se_map/set_pose")), 10);
    rest_hold_yaw_ = declare_parameter("rest_hold_yaw", rest_hold_yaw_);
    rest_hold_dist_ = declare_parameter("rest_hold_dist", rest_hold_dist_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter("wheel_odom_topic", std::string("/diff_drive_base_controller/odom")), 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& t = m->twist.twist;
          if (std::abs(t.linear.x) > 0.005 || std::abs(t.angular.z) > 0.02)
            last_motion_ns_ = now().nanoseconds();
        });
    set_pose_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (now().nanoseconds() - last_motion_ns_ < static_cast<int64_t>(1.5e9))
        rest_valid_ = false;  // moving: a new rest pose at the next rest
      if (set_pose_when_docked_)
        setPoseWhenDocked();
      if (!pose_file_.empty())
        persistPose();
      holdRestPose();
    });
  }
  if (gps_gate)
  {
    // The dock has a roof: RTK there is spotty, and float sat 1.2 m off while
    // reporting 14 mm accuracy. On the charger the dock pose is the truth, so
    // localization only gets fixes off the charger, and (by default) only RTK fixed.
    gps_require_rtk_fixed_ = declare_parameter("gps_require_rtk_fixed", true);
    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
        declare_parameter("gps_gate_out", std::string("/gps/fix_filtered")), 10);
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        declare_parameter("gps_gate_in", std::string("/gps/fix")), 10,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) { gateGps(m); });
  }
}

void open_mower_next::docking_helper::DockingHelperNode::gateGps(sensor_msgs::msg::NavSatFix::ConstSharedPtr m)
{
  std::string why;
  if (charger_present_)
    why = "on the charger";
  else if (final_approach_)
    why = "final docking approach";
  else if (gps_require_rtk_fixed_ && m->status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
    why = "no RTK fixed solution";
  if (why != gps_gate_reason_)
  {
    RCLCPP_INFO(get_logger(), "GPS to localization: %s", why.empty() ? "on" : ("off (" + why + ")").c_str());
    gps_gate_reason_ = why;
  }
  if (!why.empty())
    return;
  if (first_gps_ns_ == 0)
    first_gps_ns_ = now().nanoseconds();
  gps_pub_->publish(*m);
}

void open_mower_next::docking_helper::DockingHelperNode::setPoseWhenDocked()
{
  if (!charger_present_ || set_pose_pub_->get_subscription_count() == 0)
    return;
  const rclcpp::Time t = now();
  if (t.nanoseconds() - last_motion_ns_ < static_cast<int64_t>(3e9))
    return;  // moving (undocking): the dock pose is no longer where it stands
  // Once per stay; after that holdRestPose() keeps localization on it.
  if (!docked_anew_.exchange(false) && last_docked_pose_.nanoseconds() != 0)
    return;
  std::shared_ptr<open_mower_next::msg::DockingStation> station;
  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    if (docking_stations_.empty())
      return;
    if (docking_stations_.size() == 1)
      station = std::make_shared<open_mower_next::msg::DockingStation>(docking_stations_.front());
  }
  if (!station)
    station = findNearestDockingStation();  // by the GPS position
  std::shared_ptr<geometry_msgs::msg::PoseStamped> pose;
  try
  {
    pose = dockPose(station);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Docked, but no dock pose yet: %s", ex.what());
    return;
  }
  if (!pose)
    return;

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = now();
  msg.pose.pose = pose->pose;
  msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 * 0.02;  // x, y
  msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e-6;
  msg.pose.covariance[35] = 0.03 * 0.03;  // yaw (rad)
  set_pose_pub_->publish(msg);
  last_set_ns_ = t.nanoseconds();
  last_docked_pose_ = t;
  setRestPose(pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));
  RCLCPP_INFO(get_logger(), "Docked at '%s': localization set to x=%.3f y=%.3f yaw=%.3f", station->name.c_str(),
              pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));
}

void open_mower_next::docking_helper::DockingHelperNode::persistPose()
{
  if (!pose_restore_done_)
  {
    restorePose();
    return;
  }
  const rclcpp::Time t = now();
  if (t.nanoseconds() - last_motion_ns_ < static_cast<int64_t>(5e9))
  {
    pose_saved_ = false;  // moving, or just stopped: save at the next rest
    return;
  }
  if (pose_saved_)
    return;
  // On the charger save only after the dock pose is in localization (set in this same tick).
  if (charger_present_ && set_pose_when_docked_ &&
      (last_docked_pose_.nanoseconds() == 0 || (t - last_docked_pose_).seconds() < 0.5))
    return;
  savePose();
}

void open_mower_next::docking_helper::DockingHelperNode::savePose()
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "Pose not saved: %s", ex.what());
    return;
  }
  const double x = tf.transform.translation.x, y = tf.transform.translation.y;
  const double yaw = tf2::getYaw(tf.transform.rotation);
  // Temp file + rename: a crash never leaves half a file.
  const std::string tmp = pose_file_ + ".tmp";
  {
    std::ofstream f(tmp);
    f << std::fixed << std::setprecision(4) << x << " " << y << " " << yaw << "\n";
    if (!f)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "Pose not saved: cannot write %s", tmp.c_str());
      return;
    }
  }
  std::rename(tmp.c_str(), pose_file_.c_str());
  pose_saved_ = true;
  if (!rest_valid_)
    setRestPose(x, y, yaw);
  RCLCPP_INFO(get_logger(), "Standing still: pose saved x=%.3f y=%.3f yaw=%.3f", x, y, yaw);
}

void open_mower_next::docking_helper::DockingHelperNode::restorePose()
{
  const double elapsed = (now() - start_time_).seconds();
  // charger_present comes with the battery frame every 1.25 s
  if (!charger_known_ && elapsed < 10.0)
    return;
  // Done: pose_saved_ true = nothing to save until the robot has driven and stopped
  // (the file holds this pose already, or the heading is unknown and must not be saved).
  auto finish = [this]() { pose_restore_done_ = pose_saved_ = true; };
  if (charger_present_)
  {
    RCLCPP_INFO(get_logger(), "Saved pose not restored: on the charger, the dock pose is used");
    pose_restore_done_ = true;
    pose_saved_ = !set_pose_when_docked_;  // save the dock pose once it is set
    return;
  }
  if (last_motion_ns_ > start_time_.nanoseconds())
  {
    RCLCPP_WARN(get_logger(), "Saved pose not restored: the robot moved before it could be");
    finish();
    return;
  }
  double sx, sy, syaw;
  if (std::ifstream f(pose_file_); !(f >> sx >> sy >> syaw))
  {
    RCLCPP_INFO(get_logger(), "No saved pose in %s (heading unknown until docked or driven)", pose_file_.c_str());
    finish();
    return;
  }
  if (set_pose_pub_->get_subscription_count() == 0)
    return;  // localization not up yet
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException&)
  {
    return;
  }
  // navsat_transform starts 3 s after its first fix; give localization a few
  // seconds of RTK before comparing.
  const int64_t first_gps = first_gps_ns_;
  const bool gps = first_gps != 0 && now().nanoseconds() - first_gps > static_cast<int64_t>(8e9);
  if (!gps && elapsed < pose_restore_gps_wait_)
    return;
  double x = sx, y = sy;
  if (gps)
  {
    const double off = std::hypot(tf.transform.translation.x - sx, tf.transform.translation.y - sy);
    if (off > pose_restore_max_offset_)
    {
      RCLCPP_WARN(get_logger(), "Saved pose not restored: %.2f m from the GPS position (robot moved?)", off);
      finish();
      return;
    }
    x = tf.transform.translation.x;  // GPS is the better position; the file has the heading
    y = tf.transform.translation.y;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = now();
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, syaw);
  msg.pose.pose.orientation = tf2::toMsg(q);
  msg.pose.covariance[0] = msg.pose.covariance[7] = gps ? 0.02 * 0.02 : 0.1 * 0.1;  // x, y
  msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e-6;
  msg.pose.covariance[35] = 0.05 * 0.05;  // yaw (rad)
  set_pose_pub_->publish(msg);
  last_set_ns_ = now().nanoseconds();
  RCLCPP_INFO(get_logger(), "Saved pose restored (%s): x=%.3f y=%.3f yaw=%.3f",
              gps ? "position checked by RTK" : "no RTK fixed, unchecked", x, y, syaw);
  setRestPose(x, y, syaw);
  finish();
}

void open_mower_next::docking_helper::DockingHelperNode::setRestPose(double x, double y, double yaw)
{
  rest_x_ = x;
  rest_y_ = y;
  rest_yaw_ = yaw;
  rest_valid_ = true;
}

void open_mower_next::docking_helper::DockingHelperNode::holdRestPose()
{
  if (!rest_valid_ || set_pose_pub_->get_subscription_count() == 0 ||
      now().nanoseconds() - last_set_ns_ < static_cast<int64_t>(2e9))
    return;  // a set just sent: wait until localization shows it
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException&)
  {
    return;
  }
  const double x = tf.transform.translation.x, y = tf.transform.translation.y;
  const double dyaw = std::remainder(tf2::getYaw(tf.transform.rotation) - rest_yaw_, 2.0 * M_PI);
  // On the charger GPS is gated off: the rest position holds too. Off it GPS keeps the position.
  const bool docked = charger_present_;
  const double dist = docked ? std::hypot(x - rest_x_, y - rest_y_) : 0.0;
  if (std::abs(dyaw) <= rest_hold_yaw_ && dist <= rest_hold_dist_)
    return;
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = now();
  msg.pose.pose.position.x = docked ? rest_x_ : x;
  msg.pose.pose.position.y = docked ? rest_y_ : y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, rest_yaw_);
  msg.pose.pose.orientation = tf2::toMsg(q);
  msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 * 0.02;  // x, y
  msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e-6;
  msg.pose.covariance[35] = 0.03 * 0.03;  // yaw (rad)
  set_pose_pub_->publish(msg);
  last_set_ns_ = now().nanoseconds();
  RCLCPP_INFO(get_logger(), "Standing still: localization drifted %.1f deg / %.3f m, back to the rest pose",
              dyaw * 180.0 / M_PI, dist);
}

open_mower_next::docking_helper::DockingHelperNode::~DockingHelperNode()
{
  // No specific cleanup required
}

void open_mower_next::docking_helper::DockingHelperNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received map with %zu docking stations", msg->docking_stations.size());

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    docking_stations_.clear();

    for (const auto& docking_station : msg->docking_stations)
    {
      RCLCPP_DEBUG(get_logger(), "Docking station: %s", docking_station.name.c_str());

      docking_stations_.push_back(docking_station);
    }
  }
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStation()
{
  std::lock_guard<std::mutex> lock(docking_stations_mutex_);

  if (docking_stations_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No docking stations available");
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = this->now();
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not transform pose: %s", ex.what());
    return nullptr;
  }

  double min_distance = std::numeric_limits<double>::max();
  open_mower_next::msg::DockingStation nearest_station;

  for (const auto& station : docking_stations_)
  {
    double dx = station.pose.pose.position.x - robot_pose.pose.position.x;
    double dy = station.pose.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_station = station;
    }
  }

  RCLCPP_INFO(get_logger(), "Found nearest docking station at distance: %f meters", min_distance);
  return std::make_shared<open_mower_next::msg::DockingStation>(nearest_station);
}

void open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStationService(
    const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
    std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response)
{
  try
  {
    auto nearest_station = findNearestDockingStation();

    if (!nearest_station)
    {
      response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_NOT_FOUND;
      RCLCPP_ERROR(get_logger(), "Failed to find nearest docking station");
      return;
    }

    response->docking_station = *nearest_station;
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception occured while finding nearest docking station: %s", e.what());
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_UNKNOWN_ERROR;
    return;
  }
}

std::shared_ptr<geometry_msgs::msg::PoseStamped> open_mower_next::docking_helper::DockingHelperNode::dockPose(
    const std::shared_ptr<open_mower_next::msg::DockingStation>& station)
{
  if (!station)
  {
    RCLCPP_ERROR(get_logger(), "Cannot transform null docking station");
    return nullptr;
  }

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped->header = station->pose.header;
  pose_stamped->pose = station->pose.pose;

  geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform("base_link", "charging_port", tf2::TimePointZero);

  double offset_distance =
      std::sqrt(transform_stamped.transform.translation.x * transform_stamped.transform.translation.x +
                transform_stamped.transform.translation.y * transform_stamped.transform.translation.y);

  RCLCPP_INFO(get_logger(), "Calculated offset distance from base_link to charging_port: %f", offset_distance);

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::fromMsg(pose_stamped->pose.orientation, q_orig);
  q_rot.setRPY(0.0, 0.0, M_PI);  // Rotate 180 degrees around Z
  q_new = q_orig * q_rot;
  q_new.normalize();

  pose_stamped->pose.orientation = tf2::toMsg(q_new);

  tf2::Vector3 offset(offset_distance, 0.0, 0.0);
  tf2::Transform transform(q_new, tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Vector3 translated_offset = transform * offset;

  pose_stamped->pose.position.x -= translated_offset.x();
  pose_stamped->pose.position.y -= translated_offset.y();
  pose_stamped->pose.position.z -= translated_offset.z();

  return pose_stamped;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotNearestAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(get_logger(), "Received request to dock to nearest docking station");

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    if (docking_stations_.empty())
    {
      RCLCPP_ERROR(get_logger(), "No docking stations available");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestCancel(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to nearest station");
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT, typename GoalHandleT>
void open_mower_next::docking_helper::DockingHelperNode::executeDockingAction(
    const std::shared_ptr<GoalHandleT>& goal_handle,
    const std::shared_ptr<open_mower_next::msg::DockingStation>& docking_station)
{
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  auto result = std::make_shared<typename ActionT::Result>();

  feedback->status = ActionT::Feedback::STATUS_NONE;
  feedback->num_retries = 0;
  feedback->docking_time.sec = 0;
  feedback->docking_time.nanosec = 0;

  if (!docking_station)
  {
    result->code = ActionT::Result::CODE_DOCK_NOT_IN_DB;
    result->message = "No docking station available";
    result->num_retries = 0;
    goal_handle->abort(result);
    return;
  }

  feedback->chosen_docking_station = *docking_station;
  result->chosen_docking_station = *docking_station;

  feedback->status = ActionT::Feedback::STATUS_NAV_TO_STAGING_POSE;
  feedback->message = "Starting docking to: " + docking_station->name;
  goal_handle->publish_feedback(feedback);

  auto start_time = this->now();

  std::shared_ptr<uint16_t> current_status = std::make_shared<uint16_t>(ActionT::Feedback::STATUS_NONE);
  std::shared_ptr<uint16_t> current_retries = std::make_shared<uint16_t>(0);
  // Shared with the result callback, which may run after this function returns.
  auto docking_active = std::make_shared<std::atomic<bool>>(true);
  // Docking mode off and GPS back again however this ends (success, failure, cancel).
  struct DockingModeOff
  {
    DockingHelperNode * self;
    ~DockingModeOff()
    {
      self->final_approach_ = false;
      self->setDockingMode(false);
    }
  } docking_mode_off{this};

  auto nav2_goal = nav2_msgs::action::DockRobot::Goal();
  nav2_goal.use_dock_id = false;
  nav2_goal.navigate_to_staging_pose = true;
  nav2_goal.dock_pose.header = docking_station->pose.header;
  nav2_goal.dock_pose.pose = dockPose(docking_station)->pose;

  if (!dock_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    result->code = ActionT::Result::CODE_UNKNOWN;
    result->message = "Dock robot action server not available";
    result->num_retries = 0;
    goal_handle->abort(result);
    return;
  }

  // Send the goal
  RCLCPP_INFO(get_logger(), "Sending docking goal to station: %s", docking_station->name.c_str());
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::DockRobot>::SendGoalOptions();

  send_goal_options.feedback_callback =
      [this, current_status,
       current_retries](typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::DockRobot>::SharedPtr,
                        const std::shared_ptr<const nav2_msgs::action::DockRobot::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Docking state: %d, retries: %d", feedback->state, feedback->num_retries);
        // From the staging pose in (2 initial perception, 3 controlling, 4 waiting for
        // charge, 5 retrying = back to staging and in again): dead reckoning only.
        final_approach_ = feedback->state >= 2 && feedback->state <= 5;

        *current_status = feedback->state;
        *current_retries = feedback->num_retries;
      };

  send_goal_options.goal_response_callback = [this](const auto& goal_handle) {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Docking goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Docking goal accepted by server");
    }
  };

  send_goal_options.result_callback = [this, goal_handle, result, docking_active](const auto& nav_result) {
    auto status = nav_result.result;
    bool success = status->success;
    uint16_t error_code = status->error_code;
    uint16_t num_retries = status->num_retries;

    *docking_active = false;
    if (!goal_handle->is_active())
    {
      return;  // already canceled
    }

    result->num_retries = num_retries;

    if (success)
    {
      RCLCPP_INFO(get_logger(), "Docking action succeeded");
      result->code = ActionT::Result::CODE_SUCCESS;
      result->message = "Docking completed successfully";
      goal_handle->succeed(result);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Docking action failed with error code: %d, message: %s", error_code,
                   status->error_msg.c_str());

      result->code = error_code;
      result->message = status->error_msg.empty() ? "Docking failed" : status->error_msg;
      goal_handle->abort(result);
    }
  };

  auto nav2_goal_future = dock_client_->async_send_goal(nav2_goal, send_goal_options);

  std::string status_messages[] = { "No activity",         "Navigating to staging pose", "Initial perception of dock",
                                    "Controlling to dock", "Waiting for charge",         "Retrying docking" };

  uint16_t last_status = 99;  // Invalid value to ensure first update is sent
  uint16_t last_retries = 0;

  while (*docking_active && rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      // Propagate the cancel to Nav2's docking goal and stop.
      if (nav2_goal_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && nav2_goal_future.get())
      {
        dock_client_->async_cancel_goal(nav2_goal_future.get());
      }
      *docking_active = false;
      result->code = ActionT::Result::CODE_UNKNOWN;
      result->message = "Docking canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Docking canceled");
      return;
    }
    auto current_time = this->now();
    auto elapsed = current_time - start_time;
    feedback->docking_time.sec = elapsed.seconds();
    feedback->docking_time.nanosec = elapsed.nanoseconds() % 1000000000;

    uint16_t status = *current_status;
    uint16_t retries = *current_retries;

    if (status != last_status || retries != last_retries)
    {
      // Final approach and contact: worx_hardware pushes through the firmware's BlockForward.
      if (status != last_status) setDockingMode(status == 3 || status == 4);
      feedback->status = status;
      feedback->num_retries = retries;

      if (status < sizeof(status_messages) / sizeof(status_messages[0]))
      {
        feedback->message = status_messages[status];
        if (retries > 0)
        {
          feedback->message += " (retry " + std::to_string(retries) + ")";
        }
      }
      else
      {
        feedback->message = "Unknown status: " + std::to_string(status);
      }

      last_status = status;
      last_retries = retries;

      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestAccepted(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto nearest_station = findNearestDockingStation();
    executeDockingAction<DockRobotNearestAction>(goal_handle, nearest_station);
  } }.detach();
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findDockingStationById(const std::string& id)
{
  std::lock_guard<std::mutex> lock(docking_stations_mutex_);

  for (const auto& station : docking_stations_)
  {
    if (station.id == id)
    {
      return std::make_shared<open_mower_next::msg::DockingStation>(station);
    }
  }
  return nullptr;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotToAction::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received request to dock to station ID: %s", goal->dock_id.c_str());

  {
    std::lock_guard<std::mutex> lock(docking_stations_mutex_);
    if (docking_stations_.empty())
    {
      RCLCPP_ERROR(get_logger(), "No docking stations available");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  auto docking_station = findDockingStationById(goal->dock_id);
  if (!docking_station)
  {
    RCLCPP_ERROR(get_logger(), "Docking station with ID %s not found", goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToCancel(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to station ID: %s",
              goal_handle->get_goal()->dock_id.c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToAccepted(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto docking_station = findDockingStationById(goal->dock_id);
    executeDockingAction<DockRobotToAction>(goal_handle, docking_station);
  } }.detach();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::docking_helper::DockingHelperNode)

void open_mower_next::docking_helper::DockingHelperNode::setDockingMode(bool on)
{
  if (!docking_mode_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "/worx/docking_mode not available (docking mode %s not set)", on ? "on" : "off");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  docking_mode_client_->async_send_request(req);
}

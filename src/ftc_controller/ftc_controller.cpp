#include "ftc_controller/ftc_controller.hpp"

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <map>
#include <mutex>

namespace open_mower_next::ftc_controller
{
namespace
{
// ---- stuck recovery -------------------------------------------------------------
// The Worx firmware latches wheel speed to 0 when bumped with both wheels
// driving forward; commanding exact zero (or reversing) releases the latch.
constexpr double kStuckTimeout = 3.0;   // s commanding forward without moving
constexpr double kStuckMinMove = 0.05;  // m of movement that counts as progress
// Stuck needs this much COMMANDED travel in the window: at 20 Hz the ROS1 test
// "current command >= 0.02 m/s" fired in slow corners (0.02 m/s * 3 s < 5 cm).
constexpr double kStuckMinCommanded = 0.15;
constexpr double kReleaseTime = 1.0;    // s of exact-zero cmd (releases the latch)
constexpr double kReverseSpeed = 0.15;  // m/s straight back, no steering
constexpr double kReverseTime = 3.0;    // s -> ~0.3 m over just-driven ground
constexpr double kSkipAhead = 0.75;     // m of path skipped, times the attempt no.
constexpr int kMaxAttempts = 4;         // then give up: abort the path

// ---- turn assist (PRE_ROTATE / POST_ROTATE) ---------------------------------------
constexpr double kTurnStallTime = 3.0;
constexpr double kTurnMinProgress = 5.0 * M_PI / 180.0;
constexpr double kTurnAssistTime = 2.0;
constexpr double kTurnAssistAng = 0.4;
constexpr double kTurnReverseSpeed = 0.05;
constexpr double kTurnMaxReverse = 0.12;

std::mutex g_registry_mutex;
std::map<std::string, bool> g_finished;

int sign(double x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

Eigen::Affine3d toAffine(const geometry_msgs::msg::Pose & p)
{
  Eigen::Affine3d a;
  tf2::fromMsg(p, a);
  return a;
}
}  // namespace

void FailureDetector::update(double v, double omega, double v_max, double omega_max, double v_eps, double omega_eps)
{
  if (capacity_ == 0) return;
  M m{v, omega};
  if (m.v > 0 && v_max > 0) m.v /= v_max;
  else if (m.v < 0 && v_max > 0) m.v /= v_max;
  if (omega_max > 0) m.omega /= omega_max;
  buf_.push_back(m);
  while (buf_.size() > capacity_) buf_.pop_front();
  oscillating_ = false;
  if (buf_.size() < capacity_ / 2) return;
  double v_mean = 0, omega_mean = 0;
  int crossings = 0;
  for (size_t i = 0; i < buf_.size(); ++i) {
    v_mean += buf_[i].v;
    omega_mean += buf_[i].omega;
    if (i > 0 && sign(buf_[i].omega) != sign(buf_[i - 1].omega)) ++crossings;
  }
  v_mean /= buf_.size();
  omega_mean /= buf_.size();
  oscillating_ = std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps && crossings > 1;
}

bool FTCController::finished(const std::string & name)
{
  std::lock_guard<std::mutex> l(g_registry_mutex);
  auto it = g_finished.find(name);
  return it != g_finished.end() && it->second;
}

void FTCController::publishFinished(bool f)
{
  std::lock_guard<std::mutex> l(g_registry_mutex);
  g_finished[name_] = f;
}

double FTCController::now() const { return clock_->now().seconds(); }

void FTCController::setState(PlannerState s)
{
  state_ = s;
  state_entered_time_ = now();
  publishFinished(s == FINISHED && !is_crashed_);
}

void FTCController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) throw std::runtime_error("FTCController: node expired");
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap;
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  auto d = [&](const std::string & p, auto & value) {
    using T = std::decay_t<decltype(value)>;
    nav2_util::declare_parameter_if_not_declared(node, name + "." + p, rclcpp::ParameterValue(value));
    if constexpr (std::is_same_v<T, int>) {
      value = static_cast<int>(node->get_parameter(name + "." + p).as_int());
    } else {
      value = node->get_parameter(name + "." + p).get_value<T>();
    }
  };
  auto & c = cfg_;
  d("speed_fast", c.speed_fast); d("speed_fast_threshold", c.speed_fast_threshold);
  d("speed_fast_threshold_angle", c.speed_fast_threshold_angle); d("speed_slow", c.speed_slow);
  d("speed_angular", c.speed_angular); d("acceleration", c.acceleration);
  d("lateral_priority_distance", c.lateral_priority_distance);
  d("kp_lon", c.kp_lon); d("ki_lon", c.ki_lon); d("ki_lon_max", c.ki_lon_max); d("kd_lon", c.kd_lon);
  d("kp_lat", c.kp_lat); d("ki_lat", c.ki_lat); d("ki_lat_max", c.ki_lat_max); d("kd_lat", c.kd_lat);
  d("kp_ang", c.kp_ang); d("ki_ang", c.ki_ang); d("ki_ang_max", c.ki_ang_max); d("kd_ang", c.kd_ang);
  d("max_cmd_vel_speed", c.max_cmd_vel_speed); d("max_cmd_vel_ang", c.max_cmd_vel_ang);
  d("max_goal_distance_error", c.max_goal_distance_error); d("max_goal_angle_error", c.max_goal_angle_error);
  d("goal_timeout", c.goal_timeout); d("max_follow_distance", c.max_follow_distance);
  d("forward_only", c.forward_only); d("carrot_max_lag", c.carrot_max_lag);
  d("max_cmd_vel_accel", c.max_cmd_vel_accel); d("oscillation_recovery", c.oscillation_recovery);
  d("oscillation_v_eps", c.oscillation_v_eps); d("oscillation_omega_eps", c.oscillation_omega_eps);
  d("oscillation_recovery_min_duration", c.oscillation_recovery_min_duration);
  d("check_obstacles", c.check_obstacles); d("obstacle_footprint", c.obstacle_footprint);
  d("obstacle_lookahead", c.obstacle_lookahead);

  double controller_frequency = 20.0;
  if (node->has_parameter("controller_frequency")) {
    controller_frequency = node->get_parameter("controller_frequency").as_double();
  }
  // Buffer covers oscillation_recovery_min_duration at the real control rate.
  failure_detector_.setBufferLength(
    static_cast<size_t>(std::max(2.0, std::round(c.oscillation_recovery_min_duration * controller_frequency))));

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/global_point", 1);
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/global_plan", rclcpp::QoS(1).transient_local());
  current_movement_speed_ = c.speed_slow;
  setState(FINISHED);
  RCLCPP_INFO(logger_, "FTCController '%s' configured (speed %.2f/%.2f m/s, %.0f deg/s)", name.c_str(),
              c.speed_fast, c.speed_slow, c.speed_angular);
}

void FTCController::cleanup()
{
  carrot_pub_.reset();
  plan_pub_.reset();
}

void FTCController::activate()
{
  carrot_pub_->on_activate();
  plan_pub_->on_activate();
}

void FTCController::deactivate()
{
  carrot_pub_->on_deactivate();
  plan_pub_->on_deactivate();
}

void FTCController::reset()
{
  is_crashed_ = false;
  setState(FINISHED);
}

bool FTCController::cancel()
{
  RCLCPP_WARN(logger_, "FTC: cancelled");
  setState(FINISHED);
  return true;
}

void FTCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0 || speed_limit >= 100.0) {
    speed_limit_ = 0.0;
  } else {
    speed_limit_ = percentage ? cfg_.speed_fast * speed_limit / 100.0 : speed_limit;
  }
}

void FTCController::setPlan(const nav_msgs::msg::Path & path)
{
  is_crashed_ = false;
  plan_ = path.poses;
  plan_frame_ = path.header.frame_id.empty() ? "map" : path.header.frame_id;
  current_index_ = 0;
  current_progress_ = 0.0;
  last_time_ = now();
  current_movement_speed_ = cfg_.speed_slow;
  lat_error_ = lon_error_ = angle_error_ = 0.0;
  last_lat_error_ = last_lon_error_ = last_angle_error_ = 0.0;
  i_lon_error_ = i_lat_error_ = i_angle_error_ = 0.0;
  last_cmd_vel_linear_ = 0.0;
  carrot_gated_ = false;
  recovery_phase_ = 0;
  recovery_attempts_ = 0;
  stuck_ref_valid_ = false;
  // Oscillation state must not leak into a new plan (froze the next turn in ROS1).
  failure_detector_.clear();
  turn_ref_valid_ = false;
  turn_assist_until_ = 0.0;
  oscillation_detected_ = oscillation_warning_ = false;
  time_last_oscillation_ = now();

  if (plan_.size() > 2) {
    // duplicate the last point, give the second to last the orientation before it
    plan_.push_back(plan_.back());
    plan_[plan_.size() - 2].pose.orientation = plan_[plan_.size() - 3].pose.orientation;
    setState(PRE_ROTATE);
  } else {
    RCLCPP_WARN(logger_, "FTC: plan too short (need >= 3 poses)");
    setState(FINISHED);
  }
  if (plan_pub_ && plan_pub_->is_activated()) plan_pub_->publish(path);
  RCLCPP_INFO(logger_, "FTC: new plan with %zu poses", path.poses.size());
}

double FTCController::distanceLookahead()
{
  if (plan_.size() < 2) return 0;
  Eigen::Quaterniond current_rot(current_control_point_.linear());
  Eigen::Affine3d last_straight = current_control_point_;
  for (uint32_t i = current_index_ + 1; i < plan_.size(); i++) {
    last_straight = toAffine(plan_[i].pose);
    Eigen::Quaterniond rot2(last_straight.linear());
    if (std::abs(rot2.angularDistance(current_rot)) > cfg_.speed_fast_threshold_angle * (M_PI / 180.0)) break;
  }
  return (last_straight.translation() - current_control_point_.translation()).norm();
}

geometry_msgs::msg::TwistStamped FTCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  const double t = now();
  double dt = t - last_time_;
  last_time_ = t;
  if (dt <= 0.0 || dt > 1.0) dt = 0.05;
  robot_pos_ = Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);

  if (is_crashed_) throw nav2_core::FailedToMakeProgress("FTC: robot crashed / far from plan");
  if (state_ == FINISHED) return cmd;

  // Stuck recovery owns this cycle entirely while active.
  if (recoveryActive(robot_pos_, dt, cmd)) {
    if (is_crashed_) throw nav2_core::FailedToMakeProgress("FTC: stuck, recovery attempts exhausted");
    return cmd;
  }

  updateControlPoint(dt);
  const auto new_state = updatePlannerState();
  if (new_state != state_) {
    RCLCPP_INFO(logger_, "FTC: switching to state %d", static_cast<int>(new_state));
    turn_ref_valid_ = false;
    turn_assist_until_ = 0.0;
    setState(new_state);
  }

  if (checkCollision(cfg_.obstacle_lookahead)) {
    is_crashed_ = true;
    throw nav2_core::NoValidControl("FTC: obstacle on the path");
  }

  calculateVelocityCommands(dt, cmd);
  if (is_crashed_) throw nav2_core::FailedToMakeProgress("FTC: crashed");
  return cmd;
}

FTCController::PlannerState FTCController::updatePlannerState()
{
  switch (state_) {
    case PRE_ROTATE:
      if (timeInState() > cfg_.goal_timeout) {
        RCLCPP_ERROR(logger_, "FTC: goal_timeout (%.0f s) in PRE_ROTATE", cfg_.goal_timeout);
        is_crashed_ = true;
        return FINISHED;
      }
      if (std::abs(angle_error_) * (180.0 / M_PI) < cfg_.max_goal_angle_error) return FOLLOWING;
      break;
    case FOLLOWING: {
      const double distance = local_control_point_.translation().norm();
      if (distance > cfg_.max_follow_distance) {
        RCLCPP_ERROR(logger_, "FTC: robot %.2f m from the carrot (> max_follow_distance %.2f) - crashed?", distance,
                     cfg_.max_follow_distance);
        is_crashed_ = true;
        return FINISHED;
      }
      if (current_index_ == plan_.size() - 2) return WAITING_FOR_GOAL_APPROACH;
      break;
    }
    case WAITING_FOR_GOAL_APPROACH: {
      const double distance = local_control_point_.translation().norm();
      if (timeInState() > cfg_.goal_timeout) {
        RCLCPP_WARN(logger_, "FTC: could not reach the goal position in %.0f s - final rotation anyway", cfg_.goal_timeout);
        return POST_ROTATE;
      }
      if (distance < cfg_.max_goal_distance_error) return POST_ROTATE;
      break;
    }
    case POST_ROTATE:
      if (timeInState() > cfg_.goal_timeout) {
        RCLCPP_WARN(logger_, "FTC: could not reach the goal rotation in %.0f s", cfg_.goal_timeout);
        return FINISHED;
      }
      if (std::abs(angle_error_) * (180.0 / M_PI) < cfg_.max_goal_angle_error) return FINISHED;
      break;
    case FINISHED:
      break;
  }
  return state_;
}

void FTCController::updateControlPoint(double dt)
{
  switch (state_) {
    case PRE_ROTATE:
      current_control_point_ = toAffine(plan_[0].pose);
      break;
    case FOLLOWING: {
      const double straight_dist = distanceLookahead();
      double speed = straight_dist >= cfg_.speed_fast_threshold ? cfg_.speed_fast : cfg_.speed_slow;
      if (speed_limit_ > 0.0) speed = std::min(speed, speed_limit_);
      if (speed > current_movement_speed_) {
        current_movement_speed_ = std::min(speed, current_movement_speed_ + dt * cfg_.acceleration);
      } else if (speed < current_movement_speed_) {
        current_movement_speed_ = std::max(speed, current_movement_speed_ - dt * cfg_.acceleration);
      }
      double distance_to_move = dt * current_movement_speed_;
      double angle_to_move = dt * cfg_.speed_angular * (M_PI / 180.0);

      // Carrot leash: don't run away from a robot that fell behind.
      carrot_gated_ = false;
      if (cfg_.carrot_max_lag > 0.0) {
        const double lag = std::hypot(local_control_point_.translation().x(), local_control_point_.translation().y());
        if (lag > cfg_.carrot_max_lag) {
          distance_to_move = angle_to_move = 0.0;
          carrot_gated_ = true;
        } else if (lag > 0.5 * cfg_.carrot_max_lag) {
          const double scale = std::clamp((cfg_.carrot_max_lag - lag) / (0.5 * cfg_.carrot_max_lag), 0.0, 1.0);
          distance_to_move *= scale;
          angle_to_move *= scale;
        }
      }

      while (angle_to_move > 0 && distance_to_move > 0 && current_index_ < plan_.size() - 2) {
        const auto cur = toAffine(plan_[current_index_].pose);
        const auto next = toAffine(plan_[current_index_ + 1].pose);
        const double pose_distance = (next.translation() - cur.translation()).norm();
        const double pose_distance_angular = Eigen::Quaterniond(cur.linear()).angularDistance(Eigen::Quaterniond(next.linear()));
        if (pose_distance <= 0.0) {
          current_index_++;
          continue;
        }
        const double rem_d = pose_distance * (1.0 - current_progress_);
        const double rem_a = pose_distance_angular * (1.0 - current_progress_);
        if (rem_d < distance_to_move && rem_a < angle_to_move) {
          current_progress_ = 0.0;
          current_index_++;
          distance_to_move -= rem_d;
          angle_to_move -= rem_a;
        } else {
          const double pd = (pose_distance * current_progress_ + distance_to_move) / pose_distance;
          const double pa = pose_distance_angular > 0.0
            ? (pose_distance_angular * current_progress_ + angle_to_move) / pose_distance_angular
            : pd;
          current_progress_ = std::fmin(pa, pd);
          distance_to_move = angle_to_move = 0;
        }
      }
      const auto cur = toAffine(plan_[current_index_].pose);
      const auto next = toAffine(plan_[current_index_ + 1].pose);
      Eigen::Affine3d result = Eigen::Affine3d::Identity();
      result.translation() = (1.0 - current_progress_) * cur.translation() + current_progress_ * next.translation();
      result.linear() = Eigen::Quaterniond(cur.linear()).slerp(current_progress_, Eigen::Quaterniond(next.linear())).toRotationMatrix();
      current_control_point_ = result;
      break;
    }
    case POST_ROTATE:
      current_control_point_ = toAffine(plan_.back().pose);
      break;
    case WAITING_FOR_GOAL_APPROACH:
    case FINISHED:
      break;
  }

  if (carrot_pub_ && carrot_pub_->is_activated()) {
    geometry_msgs::msg::PoseStamped viz;
    viz.header.frame_id = plan_frame_;
    viz.header.stamp = clock_->now();
    viz.pose = tf2::toMsg(current_control_point_);
    carrot_pub_->publish(viz);
  }
  geometry_msgs::msg::TransformStamped base_from_plan;
  try {
    base_from_plan = tf_->lookupTransform(costmap_ros_->getBaseFrameID(), plan_frame_, tf2::TimePointZero,
                                          tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & e) {
    throw nav2_core::ControllerTFError(std::string("FTC: ") + e.what());
  }
  local_control_point_ = tf2::transformToEigen(base_from_plan) * current_control_point_;
  lat_error_ = local_control_point_.translation().y();
  lon_error_ = local_control_point_.translation().x();
  angle_error_ = local_control_point_.rotation().eulerAngles(0, 1, 2).z();
}

void FTCController::calculateVelocityCommands(double dt, geometry_msgs::msg::TwistStamped & cmd)
{
  if (state_ == FINISHED || is_crashed_) return;

  // Anti-windup: no longitudinal integral while the carrot is gated.
  if (!carrot_gated_) i_lon_error_ += lon_error_ * dt;
  i_lat_error_ += lat_error_ * dt;
  i_angle_error_ += angle_error_ * dt;
  i_lon_error_ = std::clamp(i_lon_error_, -cfg_.ki_lon_max, cfg_.ki_lon_max);
  i_lat_error_ = std::clamp(i_lat_error_, -cfg_.ki_lat_max, cfg_.ki_lat_max);
  i_angle_error_ = std::clamp(i_angle_error_, -cfg_.ki_ang_max, cfg_.ki_ang_max);

  const double d_lat = (lat_error_ - last_lat_error_) / dt;
  const double d_lon = (lon_error_ - last_lon_error_) / dt;
  const double d_angle = (angle_error_ - last_angle_error_) / dt;
  last_lat_error_ = lat_error_;
  last_lon_error_ = lon_error_;
  last_angle_error_ = angle_error_;

  if (state_ == FOLLOWING) {
    double lin = lon_error_ * cfg_.kp_lon + i_lon_error_ * cfg_.ki_lon + d_lon * cfg_.kd_lon;
    if (lin < 0 && cfg_.forward_only) {
      lin = 0;
    } else {
      lin = std::clamp(lin, -cfg_.max_cmd_vel_speed, cfg_.max_cmd_vel_speed);
      if (lin < 0) lat_error_ *= -1.0;
    }
    // Output slew-rate limit.
    if (cfg_.max_cmd_vel_accel > 0.0) {
      const double max_delta = cfg_.max_cmd_vel_accel * dt;
      lin = std::clamp(lin, last_cmd_vel_linear_ - max_delta, last_cmd_vel_linear_ + max_delta);
    }
    last_cmd_vel_linear_ = lin;
    cmd.twist.linear.x = lin;
  } else {
    cmd.twist.linear.x = 0.0;
    last_cmd_vel_linear_ = 0.0;
  }

  if (state_ == FOLLOWING || state_ == WAITING_FOR_GOAL_APPROACH) {
    // Reduce the angular gain while there is a large lateral error.
    double ang_gain_factor = 1.0;
    if (cfg_.lateral_priority_distance > 0.01) {
      ang_gain_factor = std::abs(lat_error_) >= cfg_.lateral_priority_distance
        ? 0.0
        : (cfg_.lateral_priority_distance - std::abs(lat_error_)) / cfg_.lateral_priority_distance;
      ang_gain_factor = std::max(ang_gain_factor, 0.1);
    }
    double ang = ang_gain_factor * (angle_error_ * cfg_.kp_ang + i_angle_error_ * cfg_.ki_ang + d_angle * cfg_.kd_ang) +
                 lat_error_ * cfg_.kp_lat + i_lat_error_ * cfg_.ki_lat + d_lat * cfg_.kd_lat;
    cmd.twist.angular.z = std::clamp(ang, -cfg_.max_cmd_vel_ang, cfg_.max_cmd_vel_ang);
  } else {
    const double ang = angle_error_ * cfg_.kp_ang + i_angle_error_ * cfg_.ki_ang + d_angle * cfg_.kd_ang;
    cmd.twist.angular.z = std::clamp(ang, -cfg_.max_cmd_vel_ang, cfg_.max_cmd_vel_ang);
    turnAssist(checkOscillation(cmd), cmd);
  }
}

bool FTCController::turnAssist(bool oscillating, geometry_msgs::msg::TwistStamped & cmd)
{
  const double t = now();
  const double err = std::fabs(angle_error_);
  if (!turn_ref_valid_) {
    turn_ref_valid_ = true;
    turn_start_pos_ = robot_pos_;
    turn_ref_error_ = err;
    turn_ref_time_ = t;
    turn_assist_until_ = 0.0;
  }
  if (turn_ref_error_ - err > kTurnMinProgress) {
    turn_ref_error_ = err;
    turn_ref_time_ = t;
  }
  const bool stalled = (t - turn_ref_time_) > kTurnStallTime;
  if ((oscillating || stalled) && t >= turn_assist_until_) {
    RCLCPP_WARN(logger_, "FTC: turn %s (error %.1f deg) - reversing slowly while turning",
                oscillating ? "oscillating" : "not progressing", err * 180.0 / M_PI);
    turn_assist_until_ = t + kTurnAssistTime;
    turn_ref_error_ = err;
    turn_ref_time_ = t;
  }
  if (t >= turn_assist_until_) return false;
  cmd.twist.angular.z = (angle_error_ >= 0.0 ? 1.0 : -1.0) * std::min(kTurnAssistAng, cfg_.max_cmd_vel_ang);
  cmd.twist.linear.x = (robot_pos_ - turn_start_pos_).norm() < kTurnMaxReverse ? -kTurnReverseSpeed : 0.0;
  return true;
}

bool FTCController::recoveryActive(const Eigen::Vector2d & p, double dt_cycle, geometry_msgs::msg::TwistStamped & cmd)
{
  const double t = now();
  if (recovery_phase_ == 0) {
    if (state_ != FOLLOWING || is_crashed_) {
      stuck_ref_valid_ = false;
      return false;
    }
    if (!stuck_ref_valid_ || (p - stuck_ref_pos_).norm() > kStuckMinMove) {
      stuck_ref_pos_ = p;
      stuck_ref_time_ = t;
      stuck_ref_valid_ = true;
      stuck_expected_travel_ = 0.0;
      return false;
    }
    stuck_expected_travel_ += std::fabs(last_cmd_vel_linear_) * dt_cycle;
    if ((t - stuck_ref_time_) < kStuckTimeout || std::fabs(last_cmd_vel_linear_) < 0.02 ||
        stuck_expected_travel_ < kStuckMinCommanded)
    {
      return false;
    }
    if (recovery_attempts_ > 0 && current_index_ - recovery_last_index_ < 30) {
      ++recovery_attempts_;
    } else {
      recovery_attempts_ = 1;
    }
    recovery_last_index_ = current_index_;
    if (recovery_attempts_ > kMaxAttempts) {
      RCLCPP_ERROR(logger_, "FTC: stuck %dx near index %u - giving up", kMaxAttempts, current_index_);
      is_crashed_ = true;
      return true;
    }
    RCLCPP_WARN(logger_, "FTC: no movement for %.0f s at index %u - recovery %d/%d (release, reverse, skip)",
                kStuckTimeout, current_index_, recovery_attempts_, kMaxAttempts);
    recovery_phase_ = 1;
    recovery_phase_start_ = t;
  }
  cmd.twist.linear.x = 0;
  cmd.twist.angular.z = 0;
  const double dt = t - recovery_phase_start_;
  switch (recovery_phase_) {
    case 1:  // exact zero: firmware latch releases
      if (dt >= kReleaseTime) { recovery_phase_ = 2; recovery_phase_start_ = t; }
      break;
    case 2:  // straight back
      cmd.twist.linear.x = -kReverseSpeed;
      if (dt >= kReverseTime) { recovery_phase_ = 3; recovery_phase_start_ = t; }
      break;
    case 3: {  // skip past the stall and resume
      const double allowed = 0.7 * cfg_.max_follow_distance - kReverseSpeed * kReverseTime;
      const double skip = std::min(kSkipAhead * recovery_attempts_, std::max(0.2, allowed));
      skipAhead(skip);
      i_lon_error_ = i_lat_error_ = i_angle_error_ = 0.0;
      last_cmd_vel_linear_ = 0.0;
      current_movement_speed_ = cfg_.speed_slow;
      stuck_ref_pos_ = p;
      stuck_ref_time_ = t;
      stuck_expected_travel_ = 0.0;
      recovery_phase_ = 0;
      RCLCPP_INFO(logger_, "FTC: recovery done, skipped %.2f m, resuming at index %u", skip, current_index_);
      return false;
    }
  }
  return true;
}

void FTCController::skipAhead(double dist)
{
  current_progress_ = 0.0;
  while (dist > 0.0 && current_index_ < plan_.size() - 2) {
    const auto & a = plan_[current_index_].pose.position;
    const auto & b = plan_[current_index_ + 1].pose.position;
    dist -= std::hypot(b.x - a.x, b.y - a.y);
    current_index_++;
  }
}

bool FTCController::checkCollision(int max_points)
{
  if (!cfg_.check_obstacles) return false;
  auto * costmap = costmap_ros_->getCostmap();
  unsigned int mx, my;
  if (cfg_.obstacle_footprint) {
    std::vector<geometry_msgs::msg::Point> footprint;
    costmap_ros_->getOrientedFootprint(footprint);
    for (const auto & pt : footprint) {
      if (costmap->worldToMap(pt.x, pt.y, mx, my) && costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE &&
          costmap->getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION)
      {
        RCLCPP_WARN(logger_, "FTC: possible collision of the footprint at the current pose");
        return true;
      }
    }
  }
  geometry_msgs::msg::TransformStamped costmap_from_plan;
  try {
    costmap_from_plan = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(), plan_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  unsigned char previous_cost = 255;
  const size_t n = std::min(plan_.size(), static_cast<size_t>(std::max(0, max_points)));
  for (size_t i = 0; i < n; i++) {
    const size_t index = std::min(plan_.size() - 1, current_index_ + i);
    geometry_msgs::msg::PoseStamped in = plan_[index], out;
    tf2::doTransform(in, out, costmap_from_plan);
    if (costmap->worldToMap(out.pose.position.x, out.pose.position.y, mx, my)) {
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost > 127 && cost > previous_cost && cost != nav2_costmap_2d::NO_INFORMATION) {
        RCLCPP_WARN(logger_, "FTC: possible collision ahead");
        return true;
      }
      previous_cost = cost;
    }
  }
  return false;
}

bool FTCController::checkOscillation(const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!cfg_.oscillation_recovery) return false;
  failure_detector_.update(cmd.twist.linear.x, cmd.twist.angular.z, cfg_.max_cmd_vel_speed, cfg_.max_cmd_vel_ang,
                           cfg_.oscillation_v_eps, cfg_.oscillation_omega_eps);
  const double t = now();
  if (failure_detector_.isOscillating()) {
    if (!oscillation_detected_) {
      time_last_oscillation_ = t;
      oscillation_detected_ = true;
    }
    if (t - time_last_oscillation_ >= cfg_.oscillation_recovery_min_duration) {
      if (!oscillation_warning_) {
        RCLCPP_WARN(logger_, "FTC: oscillation detected - turn assist");
        oscillation_warning_ = true;
      }
      return true;
    }
    return false;
  }
  time_last_oscillation_ = t;
  oscillation_detected_ = oscillation_warning_ = false;
  return false;
}

// ---- goal checker -------------------------------------------------------------------

void FTCGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
                                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".controller", rclcpp::ParameterValue(controller_));
  controller_ = node->get_parameter(plugin_name + ".controller").as_string();
}

bool FTCGoalChecker::isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
                                   const geometry_msgs::msg::Twist &)
{
  return FTCController::finished(controller_);
}

bool FTCGoalChecker::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  pose_tolerance = geometry_msgs::msg::Pose();
  vel_tolerance = geometry_msgs::msg::Twist();
  return false;
}

}  // namespace open_mower_next::ftc_controller

PLUGINLIB_EXPORT_CLASS(open_mower_next::ftc_controller::FTCController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(open_mower_next::ftc_controller::FTCGoalChecker, nav2_core::GoalChecker)

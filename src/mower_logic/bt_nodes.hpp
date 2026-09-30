#pragma once
// Behaviour tree nodes of the mower_logic executor (BehaviorTree.CPP v4).
// Tree: config/mower_logic.xml. Blade safety: only FollowPass switches the
// blade on, and it switches it off on success, failure and halt.

#include "mower_logic/context.hpp"

#include "open_mower_next/action/dock_robot_nearest.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cstdint>
#include <memory>
#include <utility>

namespace open_mower_next::mower_logic
{

using CtxPtr = std::shared_ptr<Context>;

void registerNodes(BT::BehaviorTreeFactory & factory, const CtxPtr & ctx);

// ---- generic async ROS action ------------------------------------------------------

template <class ActionT>
class RosAction : public BT::StatefulActionNode
{
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Result = typename GoalHandle::WrappedResult;

  RosAction(const std::string & name, const BT::NodeConfig & config, CtxPtr ctx, const std::string & server)
  : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), server_(server)
  {
    client_ = rclcpp_action::create_client<ActionT>(ctx_->node, server);
  }

protected:
  virtual bool makeGoal(Goal & goal) = 0;
  virtual BT::NodeStatus onResult(const Result & r)
  {
    return r.code == rclcpp_action::ResultCode::SUCCEEDED ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  virtual BT::NodeStatus whileRunning() { return BT::NodeStatus::RUNNING; }
  virtual void onCancel() {}
  virtual BT::NodeStatus beforeSend() { return BT::NodeStatus::SUCCESS; }  // RUNNING = not yet
  // Drive actions stop on a bump; the tree's bump recovery takes over.
  virtual bool abortOnBump() const { return false; }
  // Drive actions for a pass stop when the operator skipped it meanwhile.
  virtual bool staleOnSkip() const { return false; }

  BT::NodeStatus onStart() override
  {
    sent_ = false;
    state_.reset();
    started_ = Context::Clock::now();
    generation_ = ctx_->mission.generation();
    return send();
  }

  BT::NodeStatus onRunning() override
  {
    if (abortOnBump() && ctx_->bumpedSince(started_)) {
      RCLCPP_WARN(ctx_->node->get_logger(), "%s: bumped - stopping", server_.c_str());
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
    if (staleOnSkip() && ctx_->mission.generation() != generation_) {
      RCLCPP_INFO(ctx_->node->get_logger(), "%s: pass skipped by the operator - stopping", server_.c_str());
      onHalted();
      ctx_->count_failure = false;  // not a failure of the (new) current pass
      return BT::NodeStatus::FAILURE;
    }
    if (!sent_) return send();
    std::shared_ptr<State> s = state_;
    {
      std::lock_guard<std::mutex> l(s->mutex);
      if (s->rejected) {
        RCLCPP_WARN(ctx_->node->get_logger(), "%s: goal rejected", server_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (s->done) {
        const auto st = onResult(s->result);
        if (st == BT::NodeStatus::FAILURE) {
          RCLCPP_WARN(ctx_->node->get_logger(), "%s: finished with code %d", server_.c_str(), static_cast<int>(s->result.code));
        }
        return st;
      }
    }
    return whileRunning();
  }

  void onHalted() override
  {
    if (state_) {
      std::lock_guard<std::mutex> l(state_->mutex);
      if (state_->handle && !state_->done) client_->async_cancel_goal(state_->handle);
    }
    state_.reset();
    onCancel();
  }

  CtxPtr ctx_;

private:
  struct State
  {
    std::mutex mutex;
    typename GoalHandle::SharedPtr handle;
    bool rejected = false, done = false;
    Result result;
  };

  BT::NodeStatus send()
  {
    const auto pre = beforeSend();
    if (pre != BT::NodeStatus::SUCCESS) return pre;
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(ctx_->node->get_logger(), *ctx_->node->get_clock(), 5000, "%s: action server not available", server_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    Goal goal;
    if (!makeGoal(goal)) return BT::NodeStatus::FAILURE;
    auto s = std::make_shared<State>();
    typename rclcpp_action::Client<ActionT>::SendGoalOptions opt;
    opt.goal_response_callback = [s](typename GoalHandle::SharedPtr gh) {
      std::lock_guard<std::mutex> l(s->mutex);
      s->handle = gh;
      s->rejected = !gh;
    };
    opt.result_callback = [s](const Result & r) {
      std::lock_guard<std::mutex> l(s->mutex);
      s->result = r;
      s->done = true;
    };
    state_ = s;
    client_->async_send_goal(goal, opt);
    sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  std::string server_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::shared_ptr<State> state_;
  bool sent_ = false;
  Context::Clock::time_point started_{};
  unsigned generation_ = 0;
};

// ---- actions ------------------------------------------------------------------------

// Before Nav2 plans from where the robot stands: bump marks under its footprint
// are left out of the published marks, but the costmap keeps the old set until
// its next update (1 Hz) - a plan started at once could begin "in" the
// obstacle ("Start occupied"). Waits that long when something was left out.
class MarksSettle
{
public:
  BT::NodeStatus check(Context & ctx)
  {
    const auto now = Context::Clock::now();
    if (!until_) {
      if (!ctx.refreshObstacles()) return BT::NodeStatus::SUCCESS;
      until_ = now + std::chrono::milliseconds(1500);
      RCLCPP_INFO(ctx.node->get_logger(), "Bump marks under the robot left out - waiting for the costmap");
    }
    if (now < *until_) return BT::NodeStatus::RUNNING;
    until_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  void reset() { until_.reset(); }

private:
  std::optional<Context::Clock::time_point> until_;
};

// Drives to a pass start (transit controller), through a random via point on
// longer transits (Context::transitVia).
class Transit : public RosAction<nav2_msgs::action::NavigateThroughPoses>
{
public:
  Transit(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "navigate_through_poses") {}
  static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")}; }

protected:
  bool abortOnBump() const override { return true; }
  bool staleOnSkip() const override { return true; }
  BT::NodeStatus beforeSend() override { return settle_.check(*ctx_); }
  void onCancel() override
  {
    ctx_->bump_on_pass = false;
    settle_.reset();
  }
  MarksSettle settle_;
  bool makeGoal(Goal & g) override
  {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) return false;
    g.poses.clear();
    auto target = goal.value();
    std::optional<geometry_msgs::msg::PoseStamped> from = ctx_->robotPose();
    if (auto via = ctx_->transitVia(target)) {
      RCLCPP_INFO(ctx_->node->get_logger(), "Transit via (%.2f, %.2f)", via->pose.position.x, via->pose.position.y);
      g.poses.push_back(*via);
      from = via;
    }
    // The transit only has to reach the position (FTC turns to the pass heading
    // in place). Demanding the pass heading made the lattice planner search for
    // an exact arrival heading next to the edge and run out of iterations; arrive
    // heading the way we came instead.
    if (from) {
      const double yaw = std::atan2(target.pose.position.y - from->pose.position.y,
                                    target.pose.position.x - from->pose.position.x);
      target.pose.orientation.x = target.pose.orientation.y = 0.0;
      target.pose.orientation.z = std::sin(yaw / 2);
      target.pose.orientation.w = std::cos(yaw / 2);
    }
    g.poses.push_back(target);
    g.behavior_tree = ctx_->params.transit_bt;
    return true;
  }
};

// Follows one coverage pass with the blade on; reports progress to the mission.
class FollowPass : public RosAction<nav2_msgs::action::FollowPath>
{
public:
  FollowPass(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "follow_path") {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<size_t>("start_index")};
  }

protected:
  bool abortOnBump() const override { return true; }
  bool staleOnSkip() const override { return true; }
  BT::NodeStatus beforeSend() override
  {
    // Blade on first, drive after the spin-up time.
    if (!blade_since_) {
      if (!ctx_->motorsReady()) return BT::NodeStatus::RUNNING;
      ctx_->blade_in_use = true;
      ctx_->setBlade(true);
      blade_since_ = std::chrono::steady_clock::now();
    }
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - *blade_since_).count();
    return t >= ctx_->params.blade_spinup ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  bool makeGoal(Goal & g) override
  {
    auto p = getInput<nav_msgs::msg::Path>("path");
    if (!p || p.value().poses.size() < 2) return false;
    path_ = p.value();
    start_index_ = getInput<size_t>("start_index").value_or(0);
    search_from_ = 0;
    g.path = path_;
    g.controller_id = ctx_->params.controller_id;
    g.goal_checker_id = ctx_->params.goal_checker_id;
    g.progress_checker_id = ctx_->params.progress_checker_id;
    return true;
  }
  BT::NodeStatus whileRunning() override
  {
    // Progress = nearest pose in a forward window (paths can cross themselves).
    if (auto pose = ctx_->robotPose()) {
      const size_t end = std::min(path_.poses.size(), search_from_ + 60);
      double best = 1e9;
      size_t best_i = search_from_;
      for (size_t i = search_from_; i < end; ++i) {
        const auto & q = path_.poses[i].pose.position;
        const double d = std::hypot(q.x - pose->pose.position.x, q.y - pose->pose.position.y);
        if (d < best) {
          best = d;
          best_i = i;
        }
      }
      if (best < 1.0) {
        search_from_ = best_i;
        ctx_->mowed.add(ctx_->mission.setPoseIndex(start_index_ + best_i) * ctx_->params.mowed_swath);
      }
      // Past the obstacle being sidestepped: it counts as known again.
      if (ctx_->probe && ctx_->probing_obstacle == ctx_->probe->obstacle &&
          start_index_ + search_from_ > ctx_->probe->end_index) {
        ctx_->probing_obstacle = -1;
        RCLCPP_INFO(ctx_->node->get_logger(), "Past obstacle %d with the pass %.0f cm to the %s", ctx_->probe->obstacle,
                    std::abs(ctx_->probe->offset) * 100.0, ctx_->probe->offset > 0 ? "left" : "right");
      }
    }
    // FTC follows the pass and ignores the costmap: stop before a known bump
    // obstacle instead of hitting it again (outline loops pass it repeatedly).
    double along = 0.0;
    for (size_t i = search_from_; i < path_.poses.size() && along <= ctx_->params.bump_lookahead; ++i) {
      const auto & q = path_.poses[i].pose.position;
      if (auto id = ctx_->obstacleNear(q.x, q.y, ctx_->params.bump_avoid_radius)) {
        ctx_->avoiding_known_obstacle = true;
        // Shift the pass just far enough around what is left of it (smallest
        // offset first) and mow on; else skip past it with a transit.
        if (const auto off = swerve(*id, i)) {
          RCLCPP_INFO(ctx_->node->get_logger(), "Known obstacle %d ahead at (%.2f, %.2f) - swerving %.0f cm to the %s",
                      *id, q.x, q.y, std::abs(*off) * 100.0, *off > 0 ? "left" : "right");
          ctx_->continue_from_here = true;
          ctx_->skip_target.reset();  // avoid_known_obstacle: nothing to skip, the pass goes on here
          onHalted();
          return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(ctx_->node->get_logger(), "Known obstacle %d ahead at (%.2f, %.2f) - going around it", *id, q.x,
                    q.y);
        Context::Bump target;
        target.time = Context::Clock::now();
        target.x = q.x;
        target.y = q.y;
        target.obstacle = *id;
        ctx_->skip_target = target;
        ctx_->skip_counts_as_bump = false;
        ctx_->avoiding_known_obstacle = true;
        onHalted();
        return BT::NodeStatus::FAILURE;
      }
      if (i + 1 < path_.poses.size()) {
        const auto & n = path_.poses[i + 1].pose.position;
        along += std::hypot(n.x - q.x, n.y - q.y);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onResult(const Result & r) override
  {
    bladeOff();
    if (r.code != rclcpp_action::ResultCode::SUCCEEDED) return BT::NodeStatus::FAILURE;
    // A goal checker only compares with the LAST pose; passes that loop back
    // near their end (stacked perimeter loops) can "succeed" early. Accept
    // success only near the end of the path, otherwise resume from progress.
    const size_t reached = search_from_;
    if (path_.poses.size() > 20 && reached + 20 < path_.poses.size()) {
      RCLCPP_WARN(ctx_->node->get_logger(), "follow_path ended at pose %zu of %zu - resuming the rest", reached,
                  path_.poses.size());
      ctx_->count_failure = false;
      return BT::NodeStatus::FAILURE;
    }
    if (const auto resume = std::exchange(ctx_->segment_resume, std::nullopt)) {
      // Segment ended before a corner the body can't drive: continue after it.
      if (*resume == SIZE_MAX) {
        ctx_->mission.passDone();
      } else {
        ctx_->mission.continueAt(*resume);
      }
    } else {
      ctx_->mission.passDone();
    }
    ctx_->skipped_passes_in_row = 0;
    return BT::NodeStatus::SUCCESS;
  }
  void onCancel() override
  {
    bladeOff();
    ctx_->bump_on_pass = true;
  }

private:
  // Offset (m, left positive) the pass was shifted by around obstacle id near
  // pose i, the smallest that keeps bump_avoid_radius; nullopt: none fits.
  std::optional<double> swerve(int id, size_t i)
  {
    // Centre: the pass pose nearest to the obstacle within 1.5 m ahead.
    size_t c = i;
    double best = 1e9, along = 0.0;
    for (size_t k = search_from_; k < path_.poses.size() && along <= ctx_->params.bump_lookahead + 0.5; ++k) {
      const auto & q = path_.poses[k].pose.position;
      const double d = ctx_->obstacleDistance(id, q.x, q.y);
      if (d < best) {
        best = d;
        c = k;
      }
      if (k + 1 < path_.poses.size()) {
        const auto & n = path_.poses[k + 1].pose.position;
        along += std::hypot(n.x - q.x, n.y - q.y);
      }
    }
    const auto & q = path_.poses[c].pose.position;
    const double need = ctx_->params.bump_avoid_radius + 0.01;
    for (double m = 0.05; m <= ctx_->params.bump_max_swerve + 1e-9; m += 0.05) {
      for (const double sgn : {1.0, -1.0}) {
        if (ctx_->sidestep(id, q.x, q.y, sgn * m, need)) return sgn * m;
      }
    }
    return std::nullopt;
  }
  void bladeOff()
  {
    ctx_->setBlade(false);
    ctx_->blade_in_use = false;
    blade_since_.reset();
  }
  nav_msgs::msg::Path path_;
  size_t start_index_ = 0, search_from_ = 0;
  std::optional<std::chrono::steady_clock::time_point> blade_since_;
};

// Reverses a little after a bump (the local costmap checks behind the robot).
class BackUp : public RosAction<nav2_msgs::action::BackUp>
{
public:
  BackUp(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "backup") {}
  static BT::PortsList providedPorts() { return {}; }

protected:
  bool makeGoal(Goal & g) override
  {
    g.target.x = ctx_->params.bump_backup;
    g.speed = ctx_->params.bump_backup_speed;
    g.time_allowance = rclcpp::Duration::from_seconds(3.0 * ctx_->params.bump_backup / ctx_->params.bump_backup_speed);
    return true;
  }
};

class Undock : public RosAction<nav2_msgs::action::UndockRobot>
{
public:
  Undock(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "undock_robot") {}
  static BT::PortsList providedPorts() { return {}; }

protected:
  bool makeGoal(Goal & g) override
  {
    g.dock_type = ctx_->params.dock_type;
    return true;
  }
  BT::NodeStatus onResult(const Result & r) override
  {
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
      ctx_->undock_failures = 0;
      return BT::NodeStatus::SUCCESS;
    }
    const int n = ++ctx_->undock_failures;
    RCLCPP_WARN(ctx_->node->get_logger(), "Undocking failed (%d/%d)", n, ctx_->params.max_dock_attempts);
    if (n >= ctx_->params.max_dock_attempts) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "Giving up undocking; staying IDLE");
      ctx_->command = Command::IDLE;
      ctx_->undock_failures = 0;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class Dock : public RosAction<open_mower_next::action::DockRobotNearest>
{
public:
  Dock(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "dock_robot_nearest") {}
  static BT::PortsList providedPorts() { return {}; }

protected:
  bool makeGoal(Goal &) override { return true; }
  BT::NodeStatus beforeSend() override { return settle_.check(*ctx_); }
  void onCancel() override { settle_.reset(); }
  MarksSettle settle_;
  BT::NodeStatus onResult(const Result & r) override
  {
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->code == 0) {
      ctx_->dock_failures = 0;
      return BT::NodeStatus::SUCCESS;
    }
    const int n = ++ctx_->dock_failures;
    RCLCPP_WARN(ctx_->node->get_logger(), "Docking failed (%d/%d): %s", n, ctx_->params.max_dock_attempts,
                r.result ? r.result->message.c_str() : "no result");
    if (n >= ctx_->params.max_dock_attempts) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "Giving up docking; staying IDLE");
      ctx_->command = Command::IDLE;
      ctx_->dock_failures = 0;
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace open_mower_next::mower_logic

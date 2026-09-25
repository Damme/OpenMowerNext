#pragma once
// Behaviour tree nodes of the mower_logic executor (BehaviorTree.CPP v4).
// Tree: config/mower_logic.xml. Blade safety: only FollowPass switches the
// blade on, and it switches it off on success, failure and halt.

#include "mower_logic/context.hpp"

#include "open_mower_next/action/dock_robot_nearest.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>

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

  BT::NodeStatus onStart() override
  {
    sent_ = false;
    state_.reset();
    return send();
  }

  BT::NodeStatus onRunning() override
  {
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
};

// ---- actions ------------------------------------------------------------------------

class NavigateToPose : public RosAction<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPose(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "navigate_to_pose") {}
  static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")}; }

protected:
  bool makeGoal(Goal & g) override
  {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) return false;
    g.pose = goal.value();
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
  BT::NodeStatus beforeSend() override
  {
    // Blade on first, drive after the spin-up time.
    if (!blade_since_) {
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
        ctx_->mission.setPoseIndex(start_index_ + best_i);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onResult(const Result & r) override
  {
    bladeOff();
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
      ctx_->mission.passDone();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  void onCancel() override { bladeOff(); }

private:
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
};

class Dock : public RosAction<open_mower_next::action::DockRobotNearest>
{
public:
  Dock(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : RosAction(n, c, std::move(ctx), "dock_robot_nearest") {}
  static BT::PortsList providedPorts() { return {}; }

protected:
  bool makeGoal(Goal &) override { return true; }
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

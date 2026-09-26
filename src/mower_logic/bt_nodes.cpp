#include "mower_logic/bt_nodes.hpp"

#include <chrono>
#include <future>
#include <utility>

namespace open_mower_next::mower_logic
{
namespace
{
using BT::NodeStatus;

// Condition from a predicate on the context.
class Check : public BT::ConditionNode
{
public:
  Check(const std::string & n, const BT::NodeConfig & c, std::function<bool()> f)
  : BT::ConditionNode(n, c), f_(std::move(f)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override { return f_() ? NodeStatus::SUCCESS : NodeStatus::FAILURE; }

private:
  std::function<bool()> f_;
};

class CommandIs : public BT::ConditionNode
{
public:
  CommandIs(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::ConditionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("value")}; }
  NodeStatus tick() override
  {
    return getInput<std::string>("value").value_or("") == toString(ctx_->command.load()) ? NodeStatus::SUCCESS
                                                                                          : NodeStatus::FAILURE;
  }

private:
  CtxPtr ctx_;
};

class IsTrue : public BT::ConditionNode
{
public:
  IsTrue(const std::string & n, const BT::NodeConfig & c) : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<bool>("value")}; }
  NodeStatus tick() override { return getInput<bool>("value").value_or(false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE; }
};

class SetCommand : public BT::SyncActionNode
{
public:
  SetCommand(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("value")}; }
  NodeStatus tick() override
  {
    const auto v = getInput<std::string>("value").value_or("IDLE");
    ctx_->command = v == "MOW" ? Command::MOW : v == "HOME" ? Command::HOME : Command::IDLE;
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

// RUNNING forever with the blade off; marks which branch is active.
class Hold : public BT::StatefulActionNode
{
public:
  Hold(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::StatefulActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("state")}; }
  NodeStatus onStart() override
  {
    ctx_->setBlade(false);
    ctx_->setBranch(getInput<std::string>("state").value_or(name()));
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override { return NodeStatus::RUNNING; }
  void onHalted() override {}

private:
  CtxPtr ctx_;
};

class BeginMission : public BT::SyncActionNode
{
public:
  BeginMission(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    ctx_->setBranch("MOWING");
    if (ctx_->mission.active()) return NodeStatus::SUCCESS;
    const auto areas = ctx_->operationAreas();
    if (areas.empty()) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "No operation areas in the map; nothing to mow");
      ctx_->command = Command::IDLE;
      return NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ctx_->node->get_logger(), "Starting mission over %zu areas", areas.size());
    ctx_->clearBumpObstacles();
    ctx_->mission.begin(areas);
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

// Outputs the (remaining part of the) next pass; plans areas on demand.
// FAILURE when the mission is complete.
class GetPass : public BT::SyncActionNode
{
public:
  GetPass(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<nav_msgs::msg::Path>("path"), BT::OutputPort<geometry_msgs::msg::PoseStamped>("start"),
            BT::OutputPort<size_t>("start_index"), BT::OutputPort<bool>("direct")};
  }
  NodeStatus tick() override
  {
    auto & m = ctx_->mission;
    for (size_t guard = 0; guard <= m.areaCount() + 1; ++guard) {
      if (auto area = m.areaNeedingPlan()) {
        if (!plan(*area)) m.skipArea();
        continue;
      }
      break;
    }
    auto pass = m.currentPass(ctx_->params.resume_backtrack);
    if (!pass) return NodeStatus::FAILURE;
    const bool direct = startNearRobot(*pass);
    RCLCPP_INFO(ctx_->node->get_logger(), "Next: %s (%zu poses%s%s)", m.summary().c_str(), pass->path.poses.size(),
                pass->is_outline ? ", outline" : "", direct ? ", from here" : "");
    setOutput("path", pass->path);
    setOutput("start", pass->path.poses.front());
    setOutput("start_index", pass->start_index);
    setOutput("direct", direct);
    return NodeStatus::SUCCESS;
  }

private:
  // Robot already on the pass (paused by GPS loss / emergency, early end, or the
  // next pass starts where the last one ended): start at the pose next to the
  // robot and skip the transit - FTC aligns in place. The backtracked start
  // can't be used here: it lies behind the robot and FTC is forward-only, so
  // its carrot would stay gated (carrot_max_lag) and the robot would never move.
  bool startNearRobot(Mission::Pass & pass) const
  {
    const double max_d = ctx_->params.resume_direct_distance;
    if (max_d <= 0.0) return false;
    const auto pose = ctx_->robotPose();
    if (!pose) return false;
    auto & poses = pass.path.poses;
    const double search = ctx_->params.resume_backtrack + 1.0;  // m along the path
    double best = max_d, along = 0.0;
    std::optional<size_t> best_i;
    for (size_t i = 0; i + 2 < poses.size() && along <= search; ++i) {  // keep >= 3 poses for FTC
      const auto & q = poses[i].pose.position;
      const double d = std::hypot(q.x - pose->pose.position.x, q.y - pose->pose.position.y);
      if (d < best) {
        best = d;
        best_i = i;
      }
      const auto & n = poses[i + 1].pose.position;
      along += std::hypot(n.x - q.x, n.y - q.y);
    }
    if (!best_i) return false;
    poses.erase(poses.begin(), poses.begin() + static_cast<long>(*best_i));
    pass.start_index += *best_i;
    return true;
  }

  bool plan(const std::string & area)
  {
    if (!ctx_->coverage_client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "area_coverage service not available");
      return false;
    }
    auto req = std::make_shared<open_mower_next::srv::AreaCoverage::Request>();
    req->area_id = area;
    auto fut = ctx_->coverage_client->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "area_coverage timed out for %s", area.c_str());
      return false;
    }
    const auto res = fut.get();
    if (res->code != 0 || res->paths.empty()) {
      RCLCPP_ERROR(ctx_->node->get_logger(), "No coverage for %s: %s", area.c_str(), res->message.c_str());
      return false;
    }
    RCLCPP_INFO(ctx_->node->get_logger(), "Planned %s: %zu passes", area.c_str(), res->paths.size());
    ctx_->mission.setPlan(res->paths);
    return true;
  }
  CtxPtr ctx_;
};

class PassFailed : public BT::SyncActionNode
{
public:
  PassFailed(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    ctx_->setBlade(false);
    if (!ctx_->count_failure.exchange(true)) {
      return NodeStatus::SUCCESS;  // early end: just continue from the progress index
    }
    const bool skipped = ctx_->mission.passFailed(ctx_->params.max_pass_attempts);
    RCLCPP_WARN(ctx_->node->get_logger(), skipped ? "Pass failed too often - skipping it" : "Pass failed - retrying");
    if (skipped && ++ctx_->skipped_passes_in_row >= ctx_->params.max_skipped_passes_in_row) {
      // Something is wrong beyond one pass (localization, costmap, planner): don't burn the mission.
      RCLCPP_ERROR(ctx_->node->get_logger(), "%d passes in a row skipped - stopping the mission, going home",
                   ctx_->skipped_passes_in_row.load());
      ctx_->skipped_passes_in_row = 0;
      ctx_->command = Command::HOME;
    }
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

// Last resort after a refused BackUp (the footprint check at the rim, or the
// planner's heading bins clipping it, left the robot wedged against an
// obstacle): reverse straight back bump_backup m without costmap checks. FTC /
// the transit just drove forward along that line, so it is the way it came.
class ReverseAlongTrack : public BT::StatefulActionNode
{
public:
  ReverseAlongTrack(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : BT::StatefulActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus onStart() override
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "Back-up refused - reversing %.2f m along the track", ctx_->params.bump_backup);
    until_ = std::chrono::steady_clock::now() +
             std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double>(ctx_->params.bump_backup / ctx_->params.bump_backup_speed));
    return onRunning();
  }
  NodeStatus onRunning() override
  {
    if (std::chrono::steady_clock::now() >= until_) {
      ctx_->drive(0.0, 0.0);
      return NodeStatus::SUCCESS;
    }
    ctx_->drive(-ctx_->params.bump_backup_speed, 0.0);
    return NodeStatus::RUNNING;
  }
  void onHalted() override { ctx_->drive(0.0, 0.0); }

private:
  CtxPtr ctx_;
  std::chrono::steady_clock::time_point until_{};
};

// SUCCESS when a bump interrupted the pass or transit (consumes it).
class TakeBump : public BT::ConditionNode
{
public:
  TakeBump(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::ConditionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    ctx_->setBlade(false);
    const auto b = ctx_->takeBump();
    if (!b) return NodeStatus::FAILURE;
    ctx_->setBranch("BUMP_RECOVERY");
    // Bumped on a pass: continue it past the obstacle. In a transit: just retry.
    ctx_->skip_target.reset();
    if (ctx_->bump_on_pass.exchange(false)) {
      ctx_->skip_target = b;
      ctx_->skip_counts_as_bump = true;
    }
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

// SUCCESS when FollowPass stopped in front of a known bump obstacle (consumes it).
class AvoidingKnownObstacle : public BT::ConditionNode
{
public:
  AvoidingKnownObstacle(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : BT::ConditionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    const bool avoiding = ctx_->avoiding_known_obstacle;
    ctx_->avoiding_known_obstacle = false;
    if (avoiding) ctx_->bump_on_pass = false;  // FollowPass' halt set it; no bump happened
    return avoiding ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

private:
  CtxPtr ctx_;
};

// Continue the pass beyond the obstacle in skip_target (set by TakeBump after
// a bump on a pass, or by FollowPass in front of a known obstacle). The transit
// there plans around the marked obstacle.
class SkipPastBump : public BT::SyncActionNode
{
public:
  SkipPastBump(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    ctx_->setBranch("MOWING");
    const auto b = std::exchange(ctx_->skip_target, std::nullopt);
    if (!b || !std::isfinite(b->x)) return NodeStatus::SUCCESS;
    const bool on_pass = ctx_->mission.skipPastPoint(
      b->x, b->y, ctx_->params.bump_clearance, ctx_->params.max_bumps_per_pass, ctx_->skip_counts_as_bump);
    RCLCPP_WARN(ctx_->node->get_logger(), on_pass ? "Continuing past the obstacle: %s" : "Rest of the pass skipped: %s",
                ctx_->mission.summary().c_str());
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

class MissionFinished : public BT::SyncActionNode
{
public:
  MissionFinished(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    RCLCPP_INFO(ctx_->node->get_logger(), "Mowing finished - going home");
    ctx_->mission.clear();
    ctx_->command = Command::HOME;
    return NodeStatus::SUCCESS;
  }

private:
  CtxPtr ctx_;
};

template <class T>
void add(BT::BehaviorTreeFactory & f, const CtxPtr & ctx, const std::string & id)
{
  f.registerBuilder<T>(id, [ctx](const std::string & n, const BT::NodeConfig & c) { return std::make_unique<T>(n, c, ctx); });
}

void addCheck(BT::BehaviorTreeFactory & f, const std::string & id, std::function<bool()> fn)
{
  BT::TreeNodeManifest manifest;
  manifest.type = BT::NodeType::CONDITION;
  manifest.registration_ID = id;
  f.registerBuilder(manifest, [fn](const std::string & n, const BT::NodeConfig & c) { return std::make_unique<Check>(n, c, fn); });
}
}  // namespace

void registerNodes(BT::BehaviorTreeFactory & factory, const CtxPtr & ctx)
{
  addCheck(factory, "IsEmergency", [ctx]() { return ctx->emergency(); });
  addCheck(factory, "NeedsCharging", [ctx]() { return ctx->needsCharging(); });
  addCheck(factory, "IsRaining", [ctx]() { return ctx->raining(); });
  addCheck(factory, "IsDocked", [ctx]() { return ctx->charging(); });
  addCheck(factory, "IsAtDock", [ctx]() { return ctx->atDock(); });
  addCheck(factory, "GpsOK", [ctx]() { return ctx->gpsOk(); });
  factory.registerNodeType<IsTrue>("IsTrue");
  add<CommandIs>(factory, ctx, "CommandIs");
  add<SetCommand>(factory, ctx, "SetCommand");
  add<Hold>(factory, ctx, "Hold");
  add<BeginMission>(factory, ctx, "BeginMission");
  add<GetPass>(factory, ctx, "GetPass");
  add<PassFailed>(factory, ctx, "PassFailed");
  add<MissionFinished>(factory, ctx, "MissionFinished");
  add<TakeBump>(factory, ctx, "TakeBump");
  add<SkipPastBump>(factory, ctx, "SkipPastBump");
  add<AvoidingKnownObstacle>(factory, ctx, "AvoidingKnownObstacle");
  add<ReverseAlongTrack>(factory, ctx, "ReverseAlongTrack");
  add<BackUp>(factory, ctx, "BackUp");
  add<Transit>(factory, ctx, "Transit");
  add<FollowPass>(factory, ctx, "FollowPass");
  add<Undock>(factory, ctx, "Undock");
  add<Dock>(factory, ctx, "Dock");
}

}  // namespace open_mower_next::mower_logic

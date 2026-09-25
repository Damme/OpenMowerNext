#include "mower_logic/bt_nodes.hpp"

#include <chrono>
#include <future>

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
            BT::OutputPort<size_t>("start_index")};
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
    RCLCPP_INFO(ctx_->node->get_logger(), "Next: %s (%zu poses%s)", m.summary().c_str(), pass->path.poses.size(),
                pass->is_outline ? ", outline" : "");
    setOutput("path", pass->path);
    setOutput("start", pass->path.poses.front());
    setOutput("start_index", pass->start_index);
    return NodeStatus::SUCCESS;
  }

private:
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
    const bool skipped = ctx_->mission.passFailed(ctx_->params.max_pass_attempts);
    RCLCPP_WARN(ctx_->node->get_logger(), skipped ? "Pass failed too often - skipping it" : "Pass failed - retrying");
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
  addCheck(factory, "GpsOK", [ctx]() { return ctx->gpsOk(); });
  add<CommandIs>(factory, ctx, "CommandIs");
  add<SetCommand>(factory, ctx, "SetCommand");
  add<Hold>(factory, ctx, "Hold");
  add<BeginMission>(factory, ctx, "BeginMission");
  add<GetPass>(factory, ctx, "GetPass");
  add<PassFailed>(factory, ctx, "PassFailed");
  add<MissionFinished>(factory, ctx, "MissionFinished");
  add<NavigateToPose>(factory, ctx, "NavigateToPose");
  add<FollowPass>(factory, ctx, "FollowPass");
  add<Undock>(factory, ctx, "Undock");
  add<Dock>(factory, ctx, "Dock");
}

}  // namespace open_mower_next::mower_logic

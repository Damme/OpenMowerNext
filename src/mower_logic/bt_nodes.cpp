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

// Docked after a low-battery / rain return: unless auto_resume, mowing only
// continues on the next start_mowing (the mission is kept).
class EndMowingUnlessAutoResume : public BT::SyncActionNode
{
public:
  EndMowingUnlessAutoResume(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : BT::SyncActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus tick() override
  {
    if (!ctx_->params.auto_resume && ctx_->command != Command::IDLE) {
      RCLCPP_INFO(ctx_->node->get_logger(), "Docked: mowing stops here (auto_resume false), start_mowing continues %s",
                  ctx_->mission.summary().c_str());
      ctx_->command = Command::IDLE;
    }
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
      RCLCPP_ERROR(ctx_->node->get_logger(), "No operation areas to mow (map, areas, disabled_areas_file)");
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
    std::optional<Mission::Pass> pass;
    ctx_->segment_resume.reset();
    ctx_->corner_yaw.reset();
    for (int guard = 0; guard < 50; ++guard) {
      pass = m.currentPass(ctx_->params.resume_backtrack);
      if (!pass) return NodeStatus::FAILURE;
      if (cutAtUndrivable(*pass)) break;
    }
    if (!pass) return NodeStatus::FAILURE;
    const bool from_here = std::exchange(ctx_->continue_from_here, false);
    const bool direct = !std::exchange(ctx_->force_transit, false) &&
      startNearRobot(*pass, from_here ? ctx_->params.corner_max_reverse + ctx_->params.resume_direct_distance
                                      : ctx_->params.resume_direct_distance);
    RCLCPP_INFO(ctx_->node->get_logger(), "Next: %s (%zu poses%s%s)", m.summary().c_str(), pass->path.poses.size(),
                pass->is_outline ? ", outline" : "", direct ? ", from here" : "");
    ctx_->pass_is_outline = pass->is_outline;
    setOutput("path", pass->path);
    setOutput("start", pass->path.poses.front());
    setOutput("start_index", pass->start_index);
    setOutput("direct", direct);
    return NodeStatus::SUCCESS;
  }

private:
  static double yawOf(const geometry_msgs::msg::Pose & p)
  {
    const auto & q = p.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  bool fits(const geometry_msgs::msg::PoseStamped & p) const
  {
    return ctx_->footprintFits(p.pose.position.x, p.pose.position.y, yawOf(p.pose));
  }
  // Pose i fits, and so does the turn to it from pose i-1 (at a sharp vertex FTC
  // turns in place: every pose can fit with its own heading while the sweep
  // between them swings the front out).
  bool drivable(const std::vector<geometry_msgs::msg::PoseStamped> & poses, size_t i) const
  {
    if (!fits(poses[i])) return false;
    if (i == 0) return true;
    const double a = yawOf(poses[i - 1].pose);
    const double delta = std::remainder(yawOf(poses[i].pose) - a, 2.0 * M_PI);
    if (std::abs(delta) < 10.0 * M_PI / 180.0) return true;
    const int steps = static_cast<int>(std::ceil(std::abs(delta) / (5.0 * M_PI / 180.0)));
    const auto & q = poses[i].pose.position;
    for (int k = 1; k < steps; ++k) {
      if (!ctx_->footprintFits(q.x, q.y, a + delta * k / steps)) return false;
    }
    return true;
  }
  // Recorded lines can turn more sharply than the mower's body: at such corners
  // the front would leave line + rim. Drive up to the last pose that fits, then
  // continue (transit, Nav2 checks the footprint) at the next pose that fits.
  // Returns false when the pass start doesn't fit (mission moved on; ask again).
  bool cutAtUndrivable(Mission::Pass & pass)
  {
    auto & poses = pass.path.poses;
    size_t bad = poses.size();
    for (size_t i = 0; i < poses.size(); ++i) {
      if (!drivable(poses, i)) {
        bad = i;
        break;
      }
    }
    if (bad == poses.size()) return true;  // all drivable
    size_t good = bad + 1;
    while (good < poses.size() && !fits(poses[good])) ++good;
    const size_t resume = good + 3 < poses.size() ? pass.start_index + good : SIZE_MAX;
    // A segment shorter than 0.5 m isn't worth driving (FTC, forward-only, often
    // can't settle on its end in time): go straight to the corner handling.
    double seg_len = 0.0;
    for (size_t i = 1; i < bad; ++i) {
      seg_len += std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x,
                            poses[i].pose.position.y - poses[i - 1].pose.position.y);
    }
    if (bad < 3 || seg_len < 0.5) {
      // Nothing drivable before the corner: go straight to the pose after it.
      RCLCPP_WARN(ctx_->node->get_logger(), "No drivable segment before the corner - continuing at pose %zu",
                  pass.start_index + good);
      if (resume == SIZE_MAX) {
        ctx_->mission.passDone();
      } else {
        ctx_->mission.continueAt(resume);
      }
      return false;
    }
    RCLCPP_INFO(ctx_->node->get_logger(), "Corner too tight for the body at poses %zu-%zu: cut there",
                pass.start_index + bad, pass.start_index + good);
    ctx_->corner_yaw.reset();
    if (good < poses.size()) ctx_->corner_yaw = yawOf(poses[good].pose);
    poses.resize(bad);
    ctx_->segment_resume = resume;
    return true;
  }

  // Robot already on the pass (paused by GPS loss / emergency, early end, or the
  // next pass starts where the last one ended): start at the pose next to the
  // robot and skip the transit - FTC aligns in place. The backtracked start
  // can't be used here: it lies behind the robot and FTC is forward-only, so
  // its carrot would stay gated (carrot_max_lag) and the robot would never move.
  bool startNearRobot(Mission::Pass & pass, double max_d) const
  {
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
    auto passes = res->paths;
    ctx_->applyEdgeCorrections(passes);
    const auto note = ctx_->mission.setPlan(passes);
    if (!note.empty()) RCLCPP_INFO(ctx_->node->get_logger(), "%s: %s", area.c_str(), note.c_str());
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
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("distance", "m, default bump_backup")}; }
  NodeStatus onStart() override
  {
    const double d = getInput<double>("distance").value_or(ctx_->params.bump_backup);
    RCLCPP_WARN(ctx_->node->get_logger(), "Reversing %.2f m straight back the way the robot came", d);
    if (const auto pose = ctx_->robotPose()) {
      ctx_->last_reversal = Context::Reversal{Context::Clock::now(), pose->pose.position.x, pose->pose.position.y};
    }
    until_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                   std::chrono::duration<double>(d / ctx_->params.bump_backup_speed));
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

// After a segment cut before a corner the body can't take: back up as little
// as needed, turn in place to the heading after the corner with the whole
// footprint inside line + rim, then the pass continues there (FTC). Turning in
// place at the corner itself swings the front 0.47 m out. Does nothing when no
// corner is pending; FAILURE when no back-up within corner_max_reverse works
// (the continuation then goes through a transit / skips further).
// With `path` (before FollowPass): align with the pass start the same way.
// Transits end facing the travel direction and FTC's pre-rotate turned in
// place there unchecked - up to 160 deg with the front swinging out of line + rim.
class CornerTurn : public BT::StatefulActionNode
{
public:
  CornerTurn(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx) : BT::StatefulActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<nav_msgs::msg::Path>("path")}; }
  NodeStatus onStart() override
  {
    const auto path = getInput<nav_msgs::msg::Path>("path");
    align_ = path.has_value();
    std::optional<double> target;
    if (align_) {
      if (!path->poses.empty()) target = yawOf(path->poses.front());
    } else {
      target = std::exchange(ctx_->corner_yaw, std::nullopt);
    }
    const auto pose = ctx_->robotPose();
    if (!target || !pose) return NodeStatus::SUCCESS;
    target_ = *target;
    const double yaw = yawOf(*pose);
    if (std::abs(std::remainder(target_ - yaw, 2.0 * M_PI)) < 0.35) return NodeStatus::SUCCESS;  // FTC handles it
    const auto d = ctx_->reverseForTurn(pose->pose.position.x, pose->pose.position.y, yaw, target_,
                                        ctx_->params.corner_max_reverse);
    const char * what = align_ ? "Pass start" : "Corner";
    if (!d) {
      if (align_) {
        // Nothing better nearby: FTC turns as before.
        RCLCPP_WARN(ctx_->node->get_logger(), "Pass start: no back-up within %.1f m lets the body turn",
                    ctx_->params.corner_max_reverse);
        return NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(ctx_->node->get_logger(), "Corner: no back-up within %.1f m lets the body turn - transit instead",
                  ctx_->params.corner_max_reverse);
      ctx_->force_transit = true;
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(ctx_->node->get_logger(), "%s: back up %.1f m, turn %.0f deg in place", what, *d,
                std::remainder(target_ - yaw, 2.0 * M_PI) * 180.0 / M_PI);
    start_x_ = pose->pose.position.x;
    start_y_ = pose->pose.position.y;
    reverse_ = *d;
    phase_ = reverse_ > 0.0 ? Phase::REVERSE : Phase::TURN;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    return onRunning();
  }
  NodeStatus onRunning() override
  {
    const auto pose = ctx_->robotPose();
    if (!pose || std::chrono::steady_clock::now() > deadline_) {
      ctx_->drive(0.0, 0.0);
      if (!align_) ctx_->force_transit = true;
      return NodeStatus::SUCCESS;
    }
    if (phase_ == Phase::REVERSE) {
      if (std::hypot(pose->pose.position.x - start_x_, pose->pose.position.y - start_y_) >= reverse_) {
        phase_ = Phase::TURN;
      } else {
        ctx_->drive(-ctx_->params.bump_backup_speed, 0.0);
        return NodeStatus::RUNNING;
      }
    }
    const double err = std::remainder(target_ - yawOf(*pose), 2.0 * M_PI);
    if (std::abs(err) < 0.05) {
      ctx_->drive(0.0, 0.0);
      if (!align_) ctx_->continue_from_here = true;
      return NodeStatus::SUCCESS;
    }
    ctx_->drive(0.0, std::clamp(2.0 * err, -0.6, 0.6));
    return NodeStatus::RUNNING;
  }
  void onHalted() override { ctx_->drive(0.0, 0.0); }

private:
  enum class Phase { REVERSE, TURN };
  static double yawOf(const geometry_msgs::msg::PoseStamped & p)
  {
    const auto & q = p.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  CtxPtr ctx_;
  Phase phase_ = Phase::TURN;
  bool align_ = false;
  double target_ = 0.0, start_x_ = 0.0, start_y_ = 0.0, reverse_ = 0.0;
  std::chrono::steady_clock::time_point deadline_{};
};

// Before a transit or docking: when the footprint left line + rim (FTC
// overshot the end of a cut corner), Nav2 refuses every plan ("Start
// occupied") and the mission and docking die on the lawn. Back up straight
// (the way FTC came), else turn in place, until the footprint fits again.
// Always SUCCESS: the transit reports whatever is still wrong.
class FreeFootprint : public BT::StatefulActionNode
{
public:
  FreeFootprint(const std::string & n, const BT::NodeConfig & c, CtxPtr ctx)
  : BT::StatefulActionNode(n, c), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus onStart() override
  {
    const auto pose = ctx_->robotPose();
    if (!pose) return NodeStatus::SUCCESS;
    const double x = pose->pose.position.x, y = pose->pose.position.y, yaw = yawOf(*pose);
    if (ctx_->footprintFits(x, y, yaw)) return NodeStatus::SUCCESS;
    const auto e = ctx_->escapeFootprint(x, y, yaw, ctx_->params.corner_max_reverse);
    if (!e && ctx_->reversedNear(x, y)) {
      RCLCPP_WARN(ctx_->node->get_logger(),
                  "Footprint outside line + rim at (%.2f, %.2f): already reversed here, not backing up blindly again", x, y);
      return NodeStatus::FAILURE;
    }
    // Nothing fits nearby: reverse blindly the way the robot came.
    esc_ = e.value_or(Context::Escape{ctx_->params.bump_backup, 0.0});
    if (!e) ctx_->last_reversal = Context::Reversal{Context::Clock::now(), x, y};
    RCLCPP_WARN(ctx_->node->get_logger(), "Footprint outside line + rim at (%.2f, %.2f): back up %.2f m, turn %.0f deg%s",
                x, y, esc_.reverse, esc_.turn * 180.0 / M_PI, e ? "" : " (no fitting pose found)");
    start_x_ = x;
    start_y_ = y;
    target_ = yaw + esc_.turn;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    return onRunning();
  }
  NodeStatus onRunning() override
  {
    const auto pose = ctx_->robotPose();
    if (!pose || std::chrono::steady_clock::now() > deadline_) {
      ctx_->drive(0.0, 0.0);
      return NodeStatus::SUCCESS;
    }
    if (std::hypot(pose->pose.position.x - start_x_, pose->pose.position.y - start_y_) < esc_.reverse) {
      ctx_->drive(-ctx_->params.bump_backup_speed, 0.0);
      return NodeStatus::RUNNING;
    }
    const double err = std::remainder(target_ - yawOf(*pose), 2.0 * M_PI);
    if (esc_.turn == 0.0 || std::abs(err) < 0.05) {
      ctx_->drive(0.0, 0.0);
      return NodeStatus::SUCCESS;
    }
    ctx_->drive(0.0, std::clamp(2.0 * err, -0.6, 0.6));
    return NodeStatus::RUNNING;
  }
  void onHalted() override { ctx_->drive(0.0, 0.0); }

private:
  static double yawOf(const geometry_msgs::msg::PoseStamped & p)
  {
    const auto & q = p.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  CtxPtr ctx_;
  Context::Escape esc_;
  double start_x_ = 0.0, start_y_ = 0.0, target_ = 0.0;
  std::chrono::steady_clock::time_point deadline_{};
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
      // At the perimeter it is the edge (plants, GPS a few cm off), not an
      // obstacle: shift the outline inward there and keep following it.
      if (ctx_->pass_is_outline && std::isfinite(b->x)) {
        const double rx = b->x - ctx_->params.bump_front_offset * std::cos(b->yaw);
        const double ry = b->y - ctx_->params.bump_front_offset * std::sin(b->yaw);
        if (const auto off = ctx_->addEdgeCorrection(rx, ry, b->yaw)) {
          RCLCPP_WARN(ctx_->node->get_logger(), "Perimeter bump at (%.2f, %.2f): outline %.0f cm inward there (remembered)",
                      rx, ry, *off * 100.0);
          ctx_->dropBumpObstacle(*b);
          return NodeStatus::SUCCESS;  // back up, then the pass resumes a little before the spot
        }
      }
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
  add<EndMowingUnlessAutoResume>(factory, ctx, "EndMowingUnlessAutoResume");
  add<Hold>(factory, ctx, "Hold");
  add<BeginMission>(factory, ctx, "BeginMission");
  add<GetPass>(factory, ctx, "GetPass");
  add<PassFailed>(factory, ctx, "PassFailed");
  add<MissionFinished>(factory, ctx, "MissionFinished");
  add<TakeBump>(factory, ctx, "TakeBump");
  add<SkipPastBump>(factory, ctx, "SkipPastBump");
  add<AvoidingKnownObstacle>(factory, ctx, "AvoidingKnownObstacle");
  add<ReverseAlongTrack>(factory, ctx, "ReverseAlongTrack");
  add<CornerTurn>(factory, ctx, "CornerTurn");
  add<FreeFootprint>(factory, ctx, "FreeFootprint");
  add<BackUp>(factory, ctx, "BackUp");
  add<Transit>(factory, ctx, "Transit");
  add<FollowPass>(factory, ctx, "FollowPass");
  add<Undock>(factory, ctx, "Undock");
  add<Dock>(factory, ctx, "Dock");
}

}  // namespace open_mower_next::mower_logic

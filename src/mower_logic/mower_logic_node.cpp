// mower_logic: the mowing mission (undock, cover every operation area, pause
// on GPS loss/emergency, charge when low, dock when done) as a behaviour tree.
//
// Commands (std_srvs/Trigger): ~/start_mowing, ~/go_home, ~/stop (idle where it
// is), ~/skip_pass, ~/skip_area, ~/reset_mission. State: ~/state (String, 1 Hz).
// Felt obstacles and edge corrections: ~/obstacles (JSON String, latched, on
// change), ~/forget_obstacle (ForgetObstacle) to drop them.
#include "mower_logic/mower_logic_node.hpp"

#include "mower_logic/bt_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "open_mower_next/srv/forget_obstacle.hpp"

#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>

namespace open_mower_next::mower_logic
{

MowerLogicNode::MowerLogicNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("mower_logic", options))
{
  auto & node = node_;
  Params p;
  p.battery_low = node->declare_parameter("battery_low", p.battery_low);
  p.battery_resume = node->declare_parameter("battery_resume", p.battery_resume);
  p.battery_low_time = node->declare_parameter("battery_low_time", p.battery_low_time);
  p.require_gps = node->declare_parameter("require_gps", p.require_gps);
  p.gps_fix_topic = node->declare_parameter("gps_fix_topic", p.gps_fix_topic);
  p.gps_max_accuracy = node->declare_parameter("gps_max_accuracy", p.gps_max_accuracy);
  p.gps_timeout = node->declare_parameter("gps_timeout", p.gps_timeout);
  p.gps_require_rtk_fixed = node->declare_parameter("gps_require_rtk_fixed", p.gps_require_rtk_fixed);
  p.gps_settle = node->declare_parameter("gps_settle", p.gps_settle);
  p.dock_on_rain = node->declare_parameter("dock_on_rain", p.dock_on_rain);
  p.rain_clear_delay = node->declare_parameter("rain_clear_delay", p.rain_clear_delay);
  p.max_pass_attempts = static_cast<int>(node->declare_parameter("max_pass_attempts", p.max_pass_attempts));
  p.max_dock_attempts = static_cast<int>(node->declare_parameter("max_dock_attempts", p.max_dock_attempts));
  p.blade_spinup = node->declare_parameter("blade_spinup", p.blade_spinup);
  p.auto_resume = node->declare_parameter("auto_resume", p.auto_resume);
  p.mission_file = node->declare_parameter("mission_file", p.mission_file);
  p.mowed_file = node->declare_parameter("mowed_file", p.mowed_file);
  p.mowed_swath = node->declare_parameter("mowed_swath", p.mowed_swath);
  p.resume_backtrack = node->declare_parameter("resume_backtrack", p.resume_backtrack);
  p.resume_direct_distance = node->declare_parameter("resume_direct_distance", p.resume_direct_distance);
  p.bump_mark_gap = node->declare_parameter("bump_mark_gap", p.bump_mark_gap);
  p.bump_mark_depth = node->declare_parameter("bump_mark_depth", p.bump_mark_depth);
  p.bump_max_contacts = static_cast<int>(node->declare_parameter("bump_max_contacts", p.bump_max_contacts));
  p.obstacles_file = node->declare_parameter("obstacles_file", p.obstacles_file);
  p.bump_keep_free = node->declare_parameter("bump_keep_free", p.bump_keep_free);
  p.footprint_front = node->declare_parameter("footprint_front", p.footprint_front);
  p.corner_max_reverse = node->declare_parameter("corner_max_reverse", p.corner_max_reverse);
  p.footprint_rear = node->declare_parameter("footprint_rear", p.footprint_rear);
  p.footprint_half_width = node->declare_parameter("footprint_half_width", p.footprint_half_width);
  p.footprint_front_chamfer = node->declare_parameter("footprint_front_chamfer", p.footprint_front_chamfer);
  p.bump_clearance = node->declare_parameter("bump_clearance", p.bump_clearance);
  p.bump_backup = node->declare_parameter("bump_backup", p.bump_backup);
  p.bump_backup_speed = node->declare_parameter("bump_backup_speed", p.bump_backup_speed);
  p.max_bumps_per_pass = static_cast<int>(node->declare_parameter("max_bumps_per_pass", p.max_bumps_per_pass));
  p.bump_merge_distance = node->declare_parameter("bump_merge_distance", p.bump_merge_distance);
  p.bump_lookahead = node->declare_parameter("bump_lookahead", p.bump_lookahead);
  p.bump_avoid_radius = node->declare_parameter("bump_avoid_radius", p.bump_avoid_radius);
  p.feel_retry_backoff = node->declare_parameter("feel_retry_backoff", p.feel_retry_backoff);
  p.feel_sidesteps = node->declare_parameter("feel_sidesteps", p.feel_sidesteps);
  p.feel_sidestep_backoff = node->declare_parameter("feel_sidestep_backoff", p.feel_sidestep_backoff);
  p.feel_sidestep_length = node->declare_parameter("feel_sidestep_length", p.feel_sidestep_length);
  p.feel_sidestep_ramp = node->declare_parameter("feel_sidestep_ramp", p.feel_sidestep_ramp);
  p.bump_max_swerve = node->declare_parameter("bump_max_swerve", p.bump_max_swerve);
  p.feel_around = node->declare_parameter("feel_around", p.feel_around);
  p.feel_speed = node->declare_parameter("feel_speed", p.feel_speed);
  p.feel_backoff = node->declare_parameter("feel_backoff", p.feel_backoff);
  p.feel_turn = node->declare_parameter("feel_turn", p.feel_turn);
  p.feel_arc_radius = node->declare_parameter("feel_arc_radius", p.feel_arc_radius);
  p.feel_rejoin_distance = node->declare_parameter("feel_rejoin_distance", p.feel_rejoin_distance);
  p.feel_max_travel = node->declare_parameter("feel_max_travel", p.feel_max_travel);
  p.feel_max_contacts = static_cast<int>(node->declare_parameter("feel_max_contacts", p.feel_max_contacts));
  p.feel_max_offset = node->declare_parameter("feel_max_offset", p.feel_max_offset);
  p.feel_timeout = node->declare_parameter("feel_timeout", p.feel_timeout);
  p.feel_blade = node->declare_parameter("feel_blade", p.feel_blade);
  p.edge_bump_distance = node->declare_parameter("edge_bump_distance", p.edge_bump_distance);
  p.edge_correction_step = node->declare_parameter("edge_correction_step", p.edge_correction_step);
  p.edge_correction_max = node->declare_parameter("edge_correction_max", p.edge_correction_max);
  p.edge_correction_radius = node->declare_parameter("edge_correction_radius", p.edge_correction_radius);
  p.edge_correction_ramp = node->declare_parameter("edge_correction_ramp", p.edge_correction_ramp);
  p.edge_correction_ahead = node->declare_parameter("edge_correction_ahead", p.edge_correction_ahead);
  p.edge_correction_loop_distance =
    node->declare_parameter("edge_correction_loop_distance", p.edge_correction_loop_distance);
  p.edge_corrections_file = node->declare_parameter("edge_corrections_file", p.edge_corrections_file);
  p.max_skipped_passes_in_row =
    static_cast<int>(node->declare_parameter("max_skipped_passes_in_row", p.max_skipped_passes_in_row));
  p.dock_type = node->declare_parameter("dock_type", p.dock_type);
  p.undock_distance = node->declare_parameter("undock_distance", p.undock_distance);
  p.undock_heading_tolerance = node->declare_parameter("undock_heading_tolerance", p.undock_heading_tolerance);
  p.undock_lateral_tolerance = node->declare_parameter("undock_lateral_tolerance", p.undock_lateral_tolerance);
  p.transit_jitter = node->declare_parameter("transit_jitter", p.transit_jitter);
  p.transit_jitter_min_distance = node->declare_parameter("transit_jitter_min_distance", p.transit_jitter_min_distance);
  p.transit_via_margin = node->declare_parameter("transit_via_margin", p.transit_via_margin);
  p.transit_bt = node->declare_parameter(
    "transit_bt", ament_index_cpp::get_package_share_directory("open_mower_next") + "/config/behavior_trees/transit_to_pass.xml");
  p.controller_id = node->declare_parameter("controller_id", p.controller_id);
  p.goal_checker_id = node->declare_parameter("goal_checker_id", p.goal_checker_id);
  p.progress_checker_id = node->declare_parameter("progress_checker_id", p.progress_checker_id);
  {
    // Comma separated operation area ids; empty = all areas in map order.
    std::stringstream ss(node->declare_parameter("areas", std::string()));
    for (std::string id; std::getline(ss, id, ',');) {
      if (!id.empty()) p.areas.push_back(id);
    }
  }
  p.disabled_areas_file = node->declare_parameter("disabled_areas_file", p.disabled_areas_file);
  p.auto_motors = node->declare_parameter("auto_motors", p.auto_motors);
  p.motors_off_delay = node->declare_parameter("motors_off_delay", p.motors_off_delay);
  const auto tree_file = node->declare_parameter(
    "tree", ament_index_cpp::get_package_share_directory("open_mower_next") + "/config/mower_logic.xml");
  const double rate = node->declare_parameter("tick_rate", 10.0);
  const bool log_tree = node->declare_parameter("log_tree_transitions", false);

  ctx_ = std::make_shared<Context>(node, p);
  auto ctx = ctx_;

  auto trigger = [this](const std::string & name, std::function<std::string()> fn) {
    return node_->create_service<std_srvs::srv::Trigger>(
      "~/" + name, [fn, name, this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                    std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->message = fn();
        res->success = true;
        RCLCPP_INFO(node_->get_logger(), "%s: %s", name.c_str(), res->message.c_str());
      });
  };
  services_.push_back(trigger("start_mowing", [ctx]() { ctx->command = Command::MOW; return "mowing"; }));
  services_.push_back(trigger("go_home", [ctx]() { ctx->command = Command::HOME; return "going home (mission kept)"; }));
  services_.push_back(trigger("stop", [ctx]() { ctx->command = Command::IDLE; return "idle (mission kept)"; }));
  services_.push_back(trigger("skip_pass", [ctx]() { ctx->mission.skipPass(); return ctx->mission.summary(); }));
  services_.push_back(trigger("skip_area", [ctx]() { ctx->mission.skipArea(); return ctx->mission.summary(); }));
  services_.push_back(trigger("reset_mission", [ctx]() {
    ctx->mission.clear();
    ctx->clearBumpObstacles();  // Daniel: kept to look at until the mission is reset
    return "mission and felt obstacles cleared";
  }));
  services_.push_back(node_->create_service<std_srvs::srv::Trigger>(
    "~/clear_emergency", [ctx, this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = ctx->clearEmergency(res->message);
      RCLCPP_WARN(node_->get_logger(), "clear_emergency: %s", res->message.c_str());
    }));

  services_.push_back(node_->create_service<srv::ForgetObstacle>(
    "~/forget_obstacle", [ctx, this](const srv::ForgetObstacle::Request::SharedPtr req,
                                     srv::ForgetObstacle::Response::SharedPtr res) {
      const double r = req->radius > 0.0 ? req->radius : 0.5;
      using R = srv::ForgetObstacle::Request;
      if (req->kind == R::KIND_OBSTACLE || req->kind == R::KIND_ALL_OBSTACLES) {
        res->success = ctx->forgetObstacle(req->x, req->y, r, req->kind == R::KIND_ALL_OBSTACLES, res->message);
      } else if (req->kind == R::KIND_EDGE || req->kind == R::KIND_ALL_EDGES) {
        res->success = ctx->forgetEdgeCorrection(req->x, req->y, r, req->kind == R::KIND_ALL_EDGES, res->message);
      } else {
        res->success = false;
        res->message = "unknown kind '" + req->kind + "'";
      }
      RCLCPP_INFO(node_->get_logger(), "forget_obstacle %s: %s", req->kind.c_str(), res->message.c_str());
    }));

  auto state_pub = node_->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
  state_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [ctx, state_pub]() {
    std::ostringstream s;
    const double b = ctx->batteryFraction();
    s << "{\"state\":\"" << ctx->lastBranch() << "\",\"command\":\"" << toString(ctx->command.load())
      << "\",\"mission\":\"" << ctx->mission.summary() << "\",\"battery\":" << (std::isnan(b) ? -1.0 : b)
      << ",\"docked\":" << (ctx->charging() ? "true" : "false") << ",\"gps_ok\":" << (ctx->gpsOk() ? "true" : "false")
      << ",\"emergency\":" << (ctx->emergency() ? "true" : "false") << ",\"mowed\":" << ctx->mowed.json() << "}";
    std_msgs::msg::String m;
    m.data = s.str();
    state_pub->publish(m);
  });

  // Mission progress survives restarts: loaded here WITHOUT starting (command
  // stays IDLE), saved whenever it changes, removed when there is no mission.
  if (!p.mission_file.empty()) {
    std::ifstream in(p.mission_file);
    std::stringstream text;
    text << in.rdbuf();
    if (!in) {
      RCLCPP_INFO(node_->get_logger(), "No saved mission in %s", p.mission_file.c_str());
    } else if (ctx->mission.restore(text.str())) {
      RCLCPP_INFO(node_->get_logger(), "Saved mission loaded (not started, start_mowing continues it): %s",
                  ctx->mission.summary().c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Ignoring unreadable %s", p.mission_file.c_str());
    }
    saved_mission_ = ctx->mission.serialize();
    save_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this, ctx, file = p.mission_file]() {
      const auto text = ctx->mission.serialize();
      if (text == saved_mission_) return;
      if (text.empty()) {
        std::remove(file.c_str());
      } else {
        const auto tmp = file + ".tmp";
        std::ofstream(tmp) << text;
        std::rename(tmp.c_str(), file.c_str());
      }
      saved_mission_ = text;
    });
  }

  // Mowed area statistics: saved right after a charge ended, else at most once a
  // minute while mowing (one small SD write), and on shutdown.
  if (!p.mowed_file.empty()) {
    std::ifstream in(p.mowed_file);
    std::stringstream text;
    text << in.rdbuf();
    if (in && !ctx->mowed.restore(text.str())) {
      RCLCPP_WARN(node_->get_logger(), "Ignoring unreadable %s", p.mowed_file.c_str());
    }
    saved_mowed_ = ctx->mowed.serialize();
    saved_rolls_ = ctx->mowed.rolls();
    mowed_timer_ = node_->create_wall_timer(std::chrono::seconds(5), [this, ctx]() {
      const bool rolled = ctx->mowed.rolls() != saved_rolls_;
      const auto now = std::chrono::steady_clock::now();
      if (!rolled && now - mowed_saved_at_ < std::chrono::seconds(60)) return;
      saveMowed();
    });
  }

  tick_thread_ = std::thread([this, tree_file, rate, log_tree]() { run(tree_file, rate, log_tree); });
}

void MowerLogicNode::saveMowed()
{
  const auto & file = ctx_->params.mowed_file;
  if (file.empty()) return;
  saved_rolls_ = ctx_->mowed.rolls();
  mowed_saved_at_ = std::chrono::steady_clock::now();
  const auto text = ctx_->mowed.serialize();
  if (text == saved_mowed_) return;
  const auto tmp = file + ".tmp";
  std::ofstream(tmp) << text;
  std::rename(tmp.c_str(), file.c_str());
  saved_mowed_ = text;
}

MowerLogicNode::~MowerLogicNode()
{
  stop_ = true;
  if (tick_thread_.joinable()) tick_thread_.join();
  saveMowed();
}

void MowerLogicNode::run(const std::string & tree_file, double rate, bool log_tree)
{
  BT::BehaviorTreeFactory factory;
  registerNodes(factory, ctx_);
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(tree_file);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "Cannot load behaviour tree %s: %s", tree_file.c_str(), e.what());
    return;
  }
  std::unique_ptr<BT::StdCoutLogger> logger;
  if (log_tree) logger = std::make_unique<BT::StdCoutLogger>(tree);
  RCLCPP_INFO(node_->get_logger(), "mower_logic ready (%s)", tree_file.c_str());

  rclcpp::WallRate loop(rate);
  std::optional<bool> motors;  // last automatic switch (none yet)
  auto parked_since = Context::Clock::now();
  while (rclcpp::ok() && !stop_) {
    ctx_->parked = false;
    ctx_->blade_carry_seen = false;
    tree.tickOnce();
    // Belt and braces: nothing but FollowPass (and the pass chain right after
    // one, see blade_carried) may keep the blade on.
    if (!ctx_->blade_carry_seen) ctx_->blade_carried = false;
    if (!ctx_->blade_in_use && !ctx_->blade_carried) ctx_->setBlade(false);
    if (ctx_->params.auto_motors) {
      const auto now = Context::Clock::now();
      if (!ctx_->parked) parked_since = now;
      if (!ctx_->parked && motors != true) {
        if (ctx_->setMotors(true)) motors = true;
      } else if (ctx_->parked && motors != false &&
                 std::chrono::duration<double>(now - parked_since).count() >= ctx_->params.motors_off_delay) {
        if (ctx_->setMotors(false)) motors = false;
      }
    }
    loop.sleep();
  }
  tree.haltTree();
  ctx_->setBlade(false);
}

}  // namespace open_mower_next::mower_logic

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::mower_logic::MowerLogicNode)

// mower_logic: the mowing mission (undock, cover every operation area, pause
// on GPS loss/emergency, charge when low, dock when done) as a behaviour tree.
//
// Commands (std_srvs/Trigger): ~/start_mowing, ~/go_home, ~/stop (idle where it
// is), ~/skip_pass, ~/skip_area, ~/reset_mission. State: ~/state (String, 1 Hz).
#include "mower_logic/mower_logic_node.hpp"

#include "mower_logic/bt_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdio>
#include <fstream>
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
  p.resume_backtrack = node->declare_parameter("resume_backtrack", p.resume_backtrack);
  p.resume_direct_distance = node->declare_parameter("resume_direct_distance", p.resume_direct_distance);
  p.bump_front_offset = node->declare_parameter("bump_front_offset", p.bump_front_offset);
  p.bump_obstacle_radius = node->declare_parameter("bump_obstacle_radius", p.bump_obstacle_radius);
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
  services_.push_back(trigger("reset_mission", [ctx]() { ctx->mission.clear(); return "mission cleared"; }));
  services_.push_back(node_->create_service<std_srvs::srv::Trigger>(
    "~/clear_emergency", [ctx, this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = ctx->clearEmergency(res->message);
      RCLCPP_WARN(node_->get_logger(), "clear_emergency: %s", res->message.c_str());
    }));

  auto state_pub = node_->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
  state_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [ctx, state_pub]() {
    std::ostringstream s;
    const double b = ctx->batteryFraction();
    s << "{\"state\":\"" << ctx->lastBranch() << "\",\"command\":\"" << toString(ctx->command.load())
      << "\",\"mission\":\"" << ctx->mission.summary() << "\",\"battery\":" << (std::isnan(b) ? -1.0 : b)
      << ",\"docked\":" << (ctx->charging() ? "true" : "false") << ",\"gps_ok\":" << (ctx->gpsOk() ? "true" : "false")
      << ",\"emergency\":" << (ctx->emergency() ? "true" : "false") << "}";
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

  tick_thread_ = std::thread([this, tree_file, rate, log_tree]() { run(tree_file, rate, log_tree); });
}

MowerLogicNode::~MowerLogicNode()
{
  stop_ = true;
  if (tick_thread_.joinable()) tick_thread_.join();
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
  while (rclcpp::ok() && !stop_) {
    tree.tickOnce();
    // Belt and braces: nothing but FollowPass may keep the blade on.
    if (!ctx_->blade_in_use) ctx_->setBlade(false);
    loop.sleep();
  }
  tree.haltTree();
  ctx_->setBlade(false);
}

}  // namespace open_mower_next::mower_logic

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::mower_logic::MowerLogicNode)

// mower_logic: the mowing mission (undock, cover every operation area, pause
// on GPS loss/emergency, charge when low, dock when done) as a behaviour tree.
//
// Commands (std_srvs/Trigger): ~/start_mowing, ~/go_home, ~/stop (idle where it
// is), ~/skip_pass, ~/skip_area, ~/reset_mission. State: ~/state (String, 1 Hz).
#include "mower_logic/bt_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdio>
#include <sstream>
#include <thread>

using namespace open_mower_next::mower_logic;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mower_logic");

  Params p;
  p.battery_low = node->declare_parameter("battery_low", p.battery_low);
  p.battery_resume = node->declare_parameter("battery_resume", p.battery_resume);
  p.require_gps = node->declare_parameter("require_gps", p.require_gps);
  p.gps_fix_topic = node->declare_parameter("gps_fix_topic", p.gps_fix_topic);
  p.gps_max_accuracy = node->declare_parameter("gps_max_accuracy", p.gps_max_accuracy);
  p.gps_timeout = node->declare_parameter("gps_timeout", p.gps_timeout);
  p.gps_settle = node->declare_parameter("gps_settle", p.gps_settle);
  p.dock_on_rain = node->declare_parameter("dock_on_rain", p.dock_on_rain);
  p.rain_clear_delay = node->declare_parameter("rain_clear_delay", p.rain_clear_delay);
  p.max_pass_attempts = static_cast<int>(node->declare_parameter("max_pass_attempts", p.max_pass_attempts));
  p.max_dock_attempts = static_cast<int>(node->declare_parameter("max_dock_attempts", p.max_dock_attempts));
  p.blade_spinup = node->declare_parameter("blade_spinup", p.blade_spinup);
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
  const auto tree_file = node->declare_parameter(
    "tree", ament_index_cpp::get_package_share_directory("open_mower_next") + "/config/mower_logic.xml");
  const double rate = node->declare_parameter("tick_rate", 10.0);
  const bool log_tree = node->declare_parameter("log_tree_transitions", false);

  auto ctx = std::make_shared<Context>(node, p);

  auto trigger = [&node](const std::string & name, std::function<std::string()> fn) {
    return node->create_service<std_srvs::srv::Trigger>(
      "~/" + name, [fn, name, &node](const std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->message = fn();
        res->success = true;
        RCLCPP_INFO(node->get_logger(), "%s: %s", name.c_str(), res->message.c_str());
      });
  };
  auto s1 = trigger("start_mowing", [ctx]() { ctx->command = Command::MOW; return "mowing"; });
  auto s2 = trigger("go_home", [ctx]() { ctx->command = Command::HOME; return "going home (mission kept)"; });
  auto s3 = trigger("stop", [ctx]() { ctx->command = Command::IDLE; return "idle (mission kept)"; });
  auto s4 = trigger("skip_pass", [ctx]() { ctx->mission.skipPass(); return ctx->mission.summary(); });
  auto s5 = trigger("skip_area", [ctx]() { ctx->mission.skipArea(); return ctx->mission.summary(); });
  auto s6 = trigger("reset_mission", [ctx]() { ctx->mission.clear(); return "mission cleared"; });
  auto s7 = node->create_service<std_srvs::srv::Trigger>(
    "~/clear_emergency", [ctx, &node](const std_srvs::srv::Trigger::Request::SharedPtr,
                                      std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = ctx->clearEmergency(res->message);
      RCLCPP_WARN(node->get_logger(), "clear_emergency: %s", res->message.c_str());
    });

  auto state_pub = node->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
  auto state_timer = node->create_wall_timer(std::chrono::seconds(1), [&]() {
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

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  BT::BehaviorTreeFactory factory;
  registerNodes(factory, ctx);
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(tree_file);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Cannot load behaviour tree %s: %s", tree_file.c_str(), e.what());
    exec.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  std::unique_ptr<BT::StdCoutLogger> logger;
  if (log_tree) logger = std::make_unique<BT::StdCoutLogger>(tree);
  RCLCPP_INFO(node->get_logger(), "mower_logic ready (%s)", tree_file.c_str());

  rclcpp::WallRate loop(rate);
  while (rclcpp::ok()) {
    tree.tickOnce();
    // Belt and braces: nothing but FollowPass may keep the blade on.
    if (!ctx->blade_in_use) ctx->setBlade(false);
    loop.sleep();
  }
  tree.haltTree();
  ctx->setBlade(false);
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}

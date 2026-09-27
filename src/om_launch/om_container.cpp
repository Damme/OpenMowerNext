// om_container: hosts the components of one process of a launch manifest
// (see manifest.hpp), started by om_launch as `om_container <manifest> <name>`.
// Replaces component_container + the python LoadComposableNodes, and
// ros2_control_node + the python controller spawners.
//
//   - name: nav
//     ros_args: [--params-file, x.yaml]   # process-wide, e.g. for Nav2's child costmap nodes
//     components:
//       - plugin: nav2_planner::PlannerServer   # any registered rclcpp component
//         package: nav2_planner                 # optional (else all packages are searched)
//         name: planner_server
//         namespace: /                          # optional
//         params: [file.yaml, {key: value, robot_description@file: path}]
//         remap: {cmd_vel: cmd_vel_raw}
//         args: [...]                           # more node-local --ros-args
//         executor: single | multi              # default single, own thread per component
//         threads: 2                            # multi only
//       - plugin: controller_manager            # built in: ros2_control's control loop
//         name: controller_manager
//         activate: [joint_state_broadcaster, diff_drive_base_controller]
//
// Parameter files containing $(...) are expanded into /tmp first.
// `key@file: path` sets the string parameter `key` to the file's contents
// (with $(...) expanded, e.g. $(env OM_WORX_MOTORS_ENABLED true) in the URDF).
#include "om_launch/manifest.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <class_loader/class_loader.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <realtime_tools/realtime_helpers.hpp>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using open_mower_next::om_launch::loadManifest;
using open_mower_next::om_launch::readFile;
using open_mower_next::om_launch::substitute;
using open_mower_next::om_launch::dirName;

namespace
{

rclcpp::Logger logger() { return rclcpp::get_logger("om_container"); }

// ---- parameters ----------------------------------------------------------

rclcpp::ParameterValue scalarValue(const YAML::Node & n)
{
  const std::string s = n.Scalar();
  if (n.Tag() == "!") return rclcpp::ParameterValue(s);  // quoted: always a string
  if (s == "true" || s == "True") return rclcpp::ParameterValue(true);
  if (s == "false" || s == "False") return rclcpp::ParameterValue(false);
  try {
    size_t idx;
    const long long i = std::stoll(s, &idx);
    if (idx == s.size()) return rclcpp::ParameterValue(static_cast<int64_t>(i));
  } catch (...) {
  }
  try {
    size_t idx;
    const double d = std::stod(s, &idx);
    if (idx == s.size()) return rclcpp::ParameterValue(d);
  } catch (...) {
  }
  return rclcpp::ParameterValue(s);
}

rclcpp::ParameterValue sequenceValue(const YAML::Node & n)
{
  std::vector<rclcpp::ParameterValue> items;
  for (const auto & e : n) items.push_back(scalarValue(e));
  auto all = [&items](rclcpp::ParameterType t) {
    return !items.empty() && std::all_of(items.begin(), items.end(), [t](const auto & v) { return v.get_type() == t; });
  };
  if (all(rclcpp::ParameterType::PARAMETER_BOOL)) {
    std::vector<bool> v;
    for (const auto & i : items) v.push_back(i.get<bool>());
    return rclcpp::ParameterValue(v);
  }
  if (all(rclcpp::ParameterType::PARAMETER_INTEGER)) {
    std::vector<int64_t> v;
    for (const auto & i : items) v.push_back(i.get<int64_t>());
    return rclcpp::ParameterValue(v);
  }
  const bool numeric = !items.empty() && std::all_of(items.begin(), items.end(), [](const auto & i) {
    return i.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ||
           i.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
  });
  if (numeric) {
    std::vector<double> v;
    for (const auto & i : items) {
      v.push_back(i.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ? i.get<double>()
                                                                          : static_cast<double>(i.get<int64_t>()));
    }
    return rclcpp::ParameterValue(v);
  }
  std::vector<std::string> v;
  for (const auto & e : n) v.push_back(e.Scalar());
  return rclcpp::ParameterValue(v);
}

// Inline parameter map -> overrides; nested maps become dotted names.
void inlineParams(const YAML::Node & map, const std::string & prefix, std::vector<rclcpp::Parameter> & out)
{
  for (const auto & kv : map) {
    std::string key = kv.first.as<std::string>();
    const auto at = key.find("@file");
    if (at != std::string::npos && at + 5 == key.size()) {
      const auto path = kv.second.as<std::string>();
      out.emplace_back(prefix + key.substr(0, at), substitute(readFile(path), dirName(path)));
    } else if (kv.second.IsMap()) {
      inlineParams(kv.second, prefix + key + ".", out);
    } else if (kv.second.IsSequence()) {
      out.emplace_back(prefix + key, sequenceValue(kv.second));
    } else {
      out.emplace_back(prefix + key, scalarValue(kv.second));
    }
  }
}

// A parameter file with $(...) is expanded into /tmp; returns the path to use.
std::string paramFile(const std::string & path, const std::string & tag)
{
  const std::string text = readFile(path);
  if (text.find("$(") == std::string::npos) return path;
  const std::string tmp = "/tmp/om_" + tag + "_" + std::to_string(::getpid()) + "_" +
                          path.substr(path.rfind('/') + 1);
  std::ofstream(tmp) << substitute(text, dirName(path));
  return tmp;
}

// ---- component loading -----------------------------------------------------

struct Loaded
{
  std::string name;
  rclcpp_components::NodeInstanceWrapper wrapper;
  std::shared_ptr<rclcpp::Executor> executor;
  std::thread thread;
};

std::vector<std::unique_ptr<class_loader::ClassLoader>> g_loaders;

std::unique_ptr<rclcpp_components::NodeFactory> findFactory(const std::string & plugin, const std::string & package)
{
  std::vector<std::string> packages;
  if (!package.empty()) {
    packages.push_back(package);
  } else {
    for (const auto & [pkg, prefix] : ament_index_cpp::get_resources("rclcpp_components")) packages.push_back(pkg);
  }
  for (const auto & pkg : packages) {
    std::string content, prefix;
    if (!ament_index_cpp::get_resource("rclcpp_components", pkg, content, &prefix)) continue;
    std::stringstream lines(content);
    for (std::string line; std::getline(lines, line);) {
      const auto sep = line.find(';');
      if (sep == std::string::npos || line.substr(0, sep) != plugin) continue;
      std::string lib = line.substr(sep + 1);
      if (lib.empty() || lib[0] != '/') lib = prefix + "/" + lib;
      class_loader::ClassLoader * loader = nullptr;
      for (auto & l : g_loaders) {
        if (l->getLibraryPath() == lib) loader = l.get();
      }
      if (!loader) {
        g_loaders.push_back(std::make_unique<class_loader::ClassLoader>(lib));
        loader = g_loaders.back().get();
      }
      const std::string cls = "rclcpp_components::NodeFactoryTemplate<" + plugin + ">";
      for (const auto & c : loader->getAvailableClasses<rclcpp_components::NodeFactory>()) {
        if (c == cls || c == plugin) {
          return std::unique_ptr<rclcpp_components::NodeFactory>(
            loader->createUnmanagedInstance<rclcpp_components::NodeFactory>(c));
        }
      }
    }
  }
  throw std::runtime_error("component not found: " + plugin);
}

rclcpp::NodeOptions nodeOptions(const YAML::Node & c, const std::string & tag, rclcpp::NodeOptions opts = rclcpp::NodeOptions())
{
  std::vector<std::string> args{"--ros-args"};
  if (c["name"]) {
    args.push_back("-r");
    args.push_back("__node:=" + c["name"].as<std::string>());
  }
  if (c["namespace"]) {
    args.push_back("-r");
    args.push_back("__ns:=" + c["namespace"].as<std::string>());
  }
  if (c["remap"]) {
    for (const auto & kv : c["remap"]) {
      args.push_back("-r");
      args.push_back(kv.first.as<std::string>() + ":=" + kv.second.as<std::string>());
    }
  }
  std::vector<rclcpp::Parameter> overrides = opts.parameter_overrides();
  if (c["params"]) {
    for (const auto & p : c["params"]) {
      if (p.IsMap()) {
        inlineParams(p, "", overrides);
      } else {
        args.push_back("--params-file");
        args.push_back(paramFile(p.as<std::string>(), tag));
      }
    }
  }
  if (c["args"]) {
    for (const auto & a : c["args"]) args.push_back(a.as<std::string>());
  }
  auto merged = opts.arguments();
  merged.insert(merged.end(), args.begin(), args.end());
  opts.arguments(merged);
  opts.parameter_overrides(overrides);
  return opts;
}

std::shared_ptr<rclcpp::Executor> makeExecutor(const YAML::Node & c)
{
  const std::string kind = c["executor"] ? c["executor"].as<std::string>() : "single";
  if (kind == "multi") {
    const size_t threads = c["threads"] ? c["threads"].as<size_t>() : 2;
    return std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), threads);
  }
  return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

// ---- built-in: ros2_control controller manager ------------------------------
// The loop of ros2_control_node (controller_manager 4.x, periodic sleep policy),
// plus loading/activating the controllers in-process (no spawner).

class ControlManager
{
public:
  ControlManager(const YAML::Node & c, const std::string & tag)
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    const std::string name = c["name"] ? c["name"].as<std::string>() : "controller_manager";
    YAML::Node spec = YAML::Clone(c);
    spec.remove("name");  // the node name is the constructor argument
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      executor_, name, "", nodeOptions(spec, tag, controller_manager::get_cm_node_options()));
    if (c["activate"]) activate_ = c["activate"].as<std::vector<std::string>>();
    if (c["load"]) load_ = c["load"].as<std::vector<std::string>>();

    executor_->add_node(cm_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    loop_thread_ = std::thread([this]() { loop(); });
    setup_thread_ = std::thread([this]() { setupControllers(); });
  }

  ~ControlManager()
  {
    stop_ = true;
    if (setup_thread_.joinable()) setup_thread_.join();
    if (loop_thread_.joinable()) loop_thread_.join();
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    // As in ros2_control_node: the manager goes while still added to its executor.
    cm_.reset();
    executor_.reset();
  }

private:
  void loop()
  {
    const int priority = cm_->get_parameter_or<int>("thread_priority", 50);
    if (!realtime_tools::configure_sched_fifo(priority)) {
      RCLCPP_INFO(cm_->get_logger(), "No FIFO scheduling for the control loop (not permitted)");
    }
    const bool manage_overruns = cm_->get_parameter_or<bool>("overruns.manage", true);
    cm_->get_clock()->wait_until_started();
    const auto period = std::chrono::nanoseconds(1'000'000'000 / cm_->get_update_rate());
    std::this_thread::sleep_for(period);
    rclcpp::Time previous = cm_->get_trigger_clock()->now();
    auto next = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !stop_) {
      const auto now = cm_->get_trigger_clock()->now();
      const auto measured = now - previous;
      previous = now;
      cm_->read(cm_->get_trigger_clock()->now(), measured);
      cm_->update(cm_->get_trigger_clock()->now(), measured);
      cm_->write(cm_->get_trigger_clock()->now(), measured);

      next += period;
      const auto t = std::chrono::steady_clock::now();
      if (manage_overruns && next < t) {
        const double late_ms = std::chrono::duration<double, std::milli>(t - next).count();
        const double period_ms = 1e3 / cm_->get_update_rate();
        const int overruns = static_cast<int>(std::ceil(late_ms / period_ms));
        RCLCPP_WARN_THROTTLE(
          cm_->get_logger(), *cm_->get_clock(), 1000,
          "Overrun detected! The controller manager missed its desired rate of %d Hz. The loop took %f ms "
          "(missed cycles : %d).",
          cm_->get_update_rate(), late_ms + period_ms, overruns + 1);
        next += overruns * period;
      }
      std::this_thread::sleep_until(next);
    }
  }

  void setupControllers()
  {
    // The resource manager starts once robot_description has arrived.
    while (rclcpp::ok() && !stop_ && !cm_->is_resource_manager_initialized()) std::this_thread::sleep_for(100ms);
    std::vector<std::string> all = activate_;
    all.insert(all.end(), load_.begin(), load_.end());
    for (const auto & name : all) {
      if (stop_ || !rclcpp::ok()) return;
      if (!cm_->load_controller(name)) {
        RCLCPP_ERROR(cm_->get_logger(), "Cannot load controller %s", name.c_str());
        continue;
      }
      if (cm_->configure_controller(name) != controller_interface::return_type::OK) {
        RCLCPP_ERROR(cm_->get_logger(), "Cannot configure controller %s", name.c_str());
      }
    }
    if (activate_.empty() || stop_ || !rclcpp::ok()) return;
    const auto result = cm_->switch_controller(
      activate_, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT, true,
      rclcpp::Duration::from_seconds(5.0));
    if (result == controller_interface::return_type::OK) {
      RCLCPP_INFO(cm_->get_logger(), "Controllers active");
    } else {
      RCLCPP_ERROR(cm_->get_logger(), "Activating the controllers failed");
    }
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;
  std::vector<std::string> activate_, load_;
  std::atomic<bool> stop_{false};
  std::thread spin_thread_, loop_thread_, setup_thread_;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: om_container <manifest.yaml> <process name> [--ros-args ...]\n";
    return 2;
  }
  const std::string process_name = argv[2];
  YAML::Node process;
  try {
    for (const auto & p : loadManifest(argv[1])["processes"]) {
      if (p["name"].as<std::string>() == process_name) process = YAML::Node(p);
    }
  } catch (const std::exception & e) {
    std::cerr << "om_container: " << e.what() << "\n";
    return 1;
  }
  if (!process || !process["components"]) {
    std::cerr << "om_container: no process '" << process_name << "' with components\n";
    return 1;
  }

  // Process-wide ROS arguments: manifest ros_args + our own command line.
  std::vector<std::string> args{argv[0], "--ros-args"};
  if (process["ros_args"]) {
    for (const auto & a : process["ros_args"]) {
      std::string s = a.as<std::string>();
      if (!args.empty() && args.back() == "--params-file") s = paramFile(s, process_name);
      args.push_back(s);
    }
  }
  for (int i = 3; i < argc; ++i) {
    if (std::string(argv[i]) != "--ros-args") args.push_back(argv[i]);
  }
  std::vector<char *> cargs;
  for (auto & a : args) cargs.push_back(a.data());
  rclcpp::init(static_cast<int>(cargs.size()), cargs.data());

  std::vector<std::unique_ptr<Loaded>> nodes;
  std::vector<std::unique_ptr<ControlManager>> control;
  try {
    for (const auto & c : process["components"]) {
      const auto plugin = c["plugin"].as<std::string>();
      const auto name = c["name"] ? c["name"].as<std::string>() : plugin;
      if (plugin == "controller_manager") {
        control.push_back(std::make_unique<ControlManager>(c, process_name));
        RCLCPP_INFO(logger(), "Loaded %s (built-in controller manager)", name.c_str());
        continue;
      }
      auto factory = findFactory(plugin, c["package"] ? c["package"].as<std::string>() : "");
      auto loaded = std::make_unique<Loaded>();
      loaded->name = name;
      loaded->wrapper = factory->create_node_instance(nodeOptions(c, process_name));
      loaded->executor = makeExecutor(c);
      loaded->executor->add_node(loaded->wrapper.get_node_base_interface());
      auto * exec = loaded->executor.get();
      loaded->thread = std::thread([exec]() { exec->spin(); });
      RCLCPP_INFO(logger(), "Loaded %s (%s)", name.c_str(), plugin.c_str());
      nodes.push_back(std::move(loaded));
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "%s", e.what());
    rclcpp::shutdown();
    for (auto & n : nodes) n->thread.join();
    return 1;
  }

  while (rclcpp::ok()) std::this_thread::sleep_for(200ms);

  // Unload in reverse order; the controller manager (motors) goes last.
  for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
    (*it)->executor->cancel();
    if ((*it)->thread.joinable()) (*it)->thread.join();
    (*it)->executor->remove_node((*it)->wrapper.get_node_base_interface());
    (*it)->wrapper = {};
  }
  nodes.clear();
  control.clear();
  rclcpp::shutdown();
  // Component libraries stay loaded until exit: rclcpp's context still holds
  // (pre-)shutdown callbacks registered by their code (Nav2 lifecycle nodes).
  for (auto & loader : g_loaders) (void)loader.release();
  return 0;
}

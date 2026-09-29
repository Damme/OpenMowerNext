// web_ui: the robot's control page, reachable only from the local VPN (web_server.hpp).
//
// One page (index.html, compiled into the library) talks JSON over a WebSocket:
//   robot -> page  {"t":"state"} 2 Hz, {"t":"pose"} 5 Hz, {"t":"map"} on change, {"t":"log"} results/events
//   page -> robot  {"c":"<command>", ...}, see handle()
// Commands use what already exists: mower_logic's services, worx_hardware's motor enable,
// map_recorder's actions/services and map_server's remove_area. Manual driving publishes
// /cmd_vel_joy (twist_mux, top priority), only while mower_logic is IDLE, scaled to
// max_linear/max_angular here and stopped joy_timeout after the last message.
// Area on/off rewrites mower_logic's disabled_areas_file (read when a mission is planned).
// The blade is only switched by hand (Drive tab): worx_hardware's /worx/manual_mow, only
// while mower_logic is IDLE, at its runtime parameter manual_mow_pwm (Blade tab, sign =
// direction). It goes off again when a mission starts or the page that switched it on closes.
//
// All ROS callbacks run on this component's own executor thread; the web thread only
// fills inbox_, which tick() empties.
#include "web_ui/web_server.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "open_mower_next/action/record_area_boundary.hpp"
#include "open_mower_next/action/record_docking_station.hpp"
#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/msg/worx_status.hpp"
#include "open_mower_next/srv/remove_area.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>

namespace open_mower_next::web_ui
{

extern const char kIndexHtml[];

using Json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{

// Douglas-Peucker: the page only draws the areas; 2 cm is invisible and cuts
// recorded boundaries (a point every 5 cm) to a fraction.
void simplify(const std::vector<std::pair<double, double>> & p, size_t a, size_t b, double eps,
              std::vector<bool> & keep)
{
  if (b <= a + 1) return;
  const double dx = p[b].first - p[a].first, dy = p[b].second - p[a].second;
  const double len = std::hypot(dx, dy);
  double worst = -1;
  size_t idx = a;
  for (size_t i = a + 1; i < b; ++i) {
    const double ex = p[i].first - p[a].first, ey = p[i].second - p[a].second;
    const double d = len > 1e-9 ? std::abs(dx * ey - dy * ex) / len : std::hypot(ex, ey);
    if (d > worst) {
      worst = d;
      idx = i;
    }
  }
  if (worst <= eps) return;
  keep[idx] = true;
  simplify(p, a, idx, eps, keep);
  simplify(p, idx, b, eps, keep);
}

Json points(const geometry_msgs::msg::Polygon & poly, double eps)
{
  std::vector<std::pair<double, double>> p;
  p.reserve(poly.points.size());
  for (const auto & q : poly.points) p.emplace_back(q.x, q.y);
  Json out = Json::array();
  if (p.empty()) return out;
  std::vector<bool> keep(p.size(), eps <= 0);
  keep.front() = keep.back() = true;
  simplify(p, 0, p.size() - 1, eps, keep);
  for (size_t i = 0; i < p.size(); ++i) {
    // cm are enough and keep the text short
    if (keep[i]) out.push_back({std::round(p[i].first * 100) / 100, std::round(p[i].second * 100) / 100});
  }
  return out;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double r3(double v)
{
  return std::round(v * 1000) / 1000;
}

}  // namespace

class WebUiNode : public rclcpp::Node
{
public:
  explicit WebUiNode(const rclcpp::NodeOptions & options) : Node("web_ui", options)
  {
    WebServer::Options o;
    o.address = declare_parameter("bind_address", std::string("10.99.99.99"));
    o.port = static_cast<uint16_t>(declare_parameter("port", 8090));
    // Entries may also be comma separated lists (one environment variable in the manifest).
    const auto allow = declare_parameter("allow", std::vector<std::string>{"10.99.99.0/24", "10.42.40.0/22"});
    std::string allowed;
    for (const auto & entry : allow) {
      std::stringstream ss(entry);
      for (std::string net; std::getline(ss, net, ',');) {
        net.erase(0, net.find_first_not_of(" \t"));
        net.erase(net.find_last_not_of(" \t") + 1);
        if (net.empty()) continue;
        if (auto c = Cidr::parse(net)) {
          o.allow.push_back(*c);
          allowed += (allowed.empty() ? "" : ", ") + net;
        } else {
          RCLCPP_ERROR(get_logger(), "allow: ignoring '%s' (expected a.b.c.d/n)", net.c_str());
        }
      }
    }
    o.page = kIndexHtml;
    disabled_file_ = declare_parameter("disabled_areas_file", std::string());
    {
      std::stringstream ss(declare_parameter("areas", std::string()));  // mower_logic's `areas`
      for (std::string id; std::getline(ss, id, ',');) {
        if (!id.empty()) only_areas_.insert(id);
      }
    }
    max_linear_ = declare_parameter("max_linear", 0.3);
    max_angular_ = declare_parameter("max_angular", 1.0);
    joy_timeout_ = declare_parameter("joy_timeout", 0.3);

    joy_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_joy", 10);

    // The map EKF's output: cheaper than another TF listener on /tf in this process.
    pose_topic_ = declare_parameter("pose_topic", std::string("/odometry/filtered/map"));

    server_ = std::make_unique<WebServer>(
      o,
      [this](uint64_t client, std::string text) { push({client, Inbox::MESSAGE, std::move(text)}); },
      [this](uint64_t client, bool up) { push({client, up ? Inbox::UP : Inbox::DOWN, {}}); },
      [this](bool error, const std::string & text) {
        if (error) {
          RCLCPP_WARN(get_logger(), "%s", text.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", text.c_str());
        }
      });
    std::string error;
    if (server_->start(error)) {
      RCLCPP_INFO(get_logger(), "Web UI on http://%s:%u/ (allowed: %s)", o.address.c_str(), o.port,
                  allowed.empty() ? "nobody" : allowed.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Web UI not started: %s", error.c_str());
    }
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

  ~WebUiNode() override
  {
    server_.reset();  // stop the web thread before the members it calls into go
    if (driving_ && rclcpp::ok()) {
      try {
        stopDriving();
      } catch (const std::exception &) {
      }
    }
  }

private:
  using RecordArea = action::RecordAreaBoundary;
  using RecordDock = action::RecordDockingStation;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  struct Inbox
  {
    uint64_t client;
    enum Kind { MESSAGE, UP, DOWN } kind;
    std::string text;
  };

  // Subscriptions and clients exist only while a page is open: ~50 DDS endpoints
  // (and the status traffic) cost memory and CPU on the Pi for nothing otherwise.
  // Map and mower_logic state are latched, so a new page gets them at once.
  void activate()
  {
    RCLCPP_INFO(get_logger(), "Page opened: subscribing");
    logic_sub_ = create_subscription<std_msgs::msg::String>(
      "/mower_logic/state", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) {
        const auto j = Json::parse(m->data, nullptr, false);
        if (j.is_discarded() || !j.is_object()) return;
        logic_ = j;
        logic_time_ = now();
      });
    worx_sub_ = create_subscription<msg::WorxStatus>(
      "/worx/status", rclcpp::SensorDataQoS(), [this](msg::WorxStatus::ConstSharedPtr m) {
        worx_ = {{"link", m->link_ok},        {"emergency", m->emergency}, {"lift", m->lift},
                 {"collision", m->collision}, {"bumps", m->bumps},         {"motors", m->motors_enabled},
                 {"power", m->power_state},   {"v", r3(m->battery_voltage)},
                 {"a", r3(m->battery_current)},  {"in_charger", m->in_charger},
                 {"pwm", {m->motor_pwm[0], m->motor_pwm[1], m->motor_pwm[2]}},
                 {"blade", {{"manual", m->manual_mow},        {"pwm", m->manual_mow_pwm},
                            {"max", m->manual_mow_max_pwm},   {"cmd", m->mow_pwm_cmd},
                            {"current", m->motor_current[2]}, {"pulses", m->mow_pulses}}}};
        for (size_t i = 0; i < m->digital_names.size() && i < m->digital_active.size(); ++i) {
          if (m->digital_names[i] == "Rain") worx_["rain"] = static_cast<bool>(m->digital_active[i]);
        }
        worx_time_ = now();
      });
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        gps_status_ = m->status.status;
        gps_acc_ = std::sqrt(std::max(0.0, m->position_covariance[0]));
        gps_time_ = now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        pose_ = {m->pose.pose.position.x, m->pose.pose.position.y, yawOf(m->pose.pose.orientation)};
        have_pose_ = true;
      });
    map_sub_ = create_subscription<msg::Map>(
      "/mowing_map", rclcpp::QoS(1).transient_local().reliable(), [this](msg::Map::ConstSharedPtr m) {
        map_ = m;
        sendMap(0);
      });

    for (const char * name :
         {"start_mowing", "go_home", "stop", "skip_pass", "skip_area", "reset_mission", "clear_emergency"}) {
      logic_clients_[name] = create_client<std_srvs::srv::Trigger>(std::string("/mower_logic/") + name);
    }
    motors_client_ = create_client<std_srvs::srv::SetBool>("/worx/motors_enabled");
    if (!blade_client_) blade_client_ = create_client<std_srvs::srv::SetBool>("/worx/manual_mow");
    worx_params_client_ = create_client<rcl_interfaces::srv::SetParameters>("/worx_hardware/set_parameters");
    rec_mode_client_ = create_client<std_srvs::srv::SetBool>("/set_recording_mode");
    rec_point_client_ = create_client<std_srvs::srv::Trigger>("/add_boundary_point");
    rec_finish_client_ = create_client<std_srvs::srv::Trigger>("/finish_area_recording");
    remove_area_client_ = create_client<srv::RemoveArea>("/remove_area");
    if (!rec_area_client_) rec_area_client_ = rclcpp_action::create_client<RecordArea>(this, "/record_area_boundary");
    if (!rec_dock_client_) rec_dock_client_ = rclcpp_action::create_client<RecordDock>(this, "/record_docking_station");
  }

  void deactivate()
  {
    RCLCPP_INFO(get_logger(), "Last page closed: unsubscribing");
    logic_sub_.reset();
    worx_sub_.reset();
    gps_sub_.reset();
    odom_sub_.reset();
    map_sub_.reset();
    logic_clients_.clear();
    motors_client_.reset();
    worx_params_client_.reset();
    rec_mode_client_.reset();
    rec_point_client_.reset();
    rec_finish_client_.reset();
    remove_area_client_.reset();
    logic_ = worx_ = Json();
    map_.reset();
    have_pose_ = false;
    // Action clients stay while a recording runs (a page opened later can finish it); tick() drops them after.
    // So does the blade client until its last request (switching the blade off) is answered.
  }

  void push(Inbox && e)
  {
    std::lock_guard<std::mutex> l(inbox_mutex_);
    if (inbox_.size() < 200) inbox_.push_back(std::move(e));
  }

  void tick()
  {
    std::vector<Inbox> events;
    {
      std::lock_guard<std::mutex> l(inbox_mutex_);
      events.swap(inbox_);
    }
    for (auto & e : events) {
      if (e.kind == Inbox::UP) {
        if (pages_.empty()) activate();
        pages_.insert(e.client);
        hello(e.client);
      } else if (e.kind == Inbox::DOWN) {
        if (driving_ && e.client == joy_client_) {
          stopDriving();
          event(false, "Page closed while driving: stopped");
        }
        if (bladeRunning() && e.client == blade_page_) {
          setBlade(false);
          event(false, "Page closed with the blade on: blade off");
        }
        if (pages_.erase(e.client) && pages_.empty()) deactivate();
      } else if (pages_.count(e.client)) {
        try {
          handle(e.client, Json::parse(e.text));
        } catch (const Json::exception &) {
          send(e.client, logLine(false, "bad message"));
        }
      }
    }
    if (driving_ && (now() - last_joy_).seconds() > joy_timeout_) stopDriving();
    bladeWatch();
    if (pages_.empty()) {
      if (blade_client_ && blade_requests_ == 0) blade_client_.reset();
      if (rec_area_client_ && !rec_goal_ && !rec_pending_) rec_area_client_.reset();
      if (rec_dock_client_ && !dock_goal_ && !dock_pending_) rec_dock_client_.reset();
    }

    ++ticks_;
    if (pages_.empty()) return;
    if (ticks_ % 4 == 0 && have_pose_) {
      server_->send(0, Json{{"t", "pose"}, {"x", r3(pose_[0])}, {"y", r3(pose_[1])}, {"yaw", r3(pose_[2])}}.dump());
    }
    if (ticks_ % 10 == 0) server_->send(0, state().dump());
    if (ticks_ % 20 == 0 && rec_dirty_) {
      rec_dirty_ = false;
      server_->send(0, Json{{"t", "rec"}, {"pts", rec_points_}}.dump());
    }
  }

  void hello(uint64_t client)
  {
    sendMap(client);
    server_->send(client, state().dump());
    for (const auto & line : log_) server_->send(client, line);
    if (!rec_points_.empty()) server_->send(client, Json{{"t", "rec"}, {"pts", rec_points_}}.dump());
  }

  void send(uint64_t client, const std::string & text)
  {
    if (server_) server_->send(client, text);
  }

  // ---- commands --------------------------------------------------------------

  void handle(uint64_t client, const Json & m)
  {
    const std::string c = m.at("c").get<std::string>();
    if (c == "joy") {
      joy(client, m.at("v").get<double>(), m.at("w").get<double>());
    } else if (c == "logic") {
      const auto it = logic_clients_.find(m.at("name").get<std::string>());
      if (it == logic_clients_.end()) {
        send(client, logLine(false, "unknown mower_logic command"));
        return;
      }
      if (it->first == "start_mowing" || it->first == "go_home") {
        if (driving_) stopDriving();
        if (bladeRunning()) setBlade(false);
      }
      trigger(it->second, it->first);
    } else if (c == "motors") {
      setBool(motors_client_, m.at("on").get<bool>(), "Motors");
    } else if (c == "blade") {
      blade(client, m.at("on").get<bool>());
    } else if (c == "blade_pwm") {
      setBladePwm(m.at("pwm").get<int>());
    } else if (c == "area") {
      setAreaEnabled(m.at("id").get<std::string>(), m.at("enabled").get<bool>());
    } else if (c == "area_remove") {
      removeArea(m.at("id").get<std::string>());
    } else if (c == "rec_start") {
      recordArea(m.at("name").get<std::string>(), m.at("type").get<int>(), m.at("auto").get<bool>());
    } else if (c == "rec_auto") {
      setBool(rec_mode_client_, m.at("auto").get<bool>(), "Automatic points");
    } else if (c == "rec_point") {
      trigger(rec_point_client_, "Add point");
    } else if (c == "rec_finish") {
      trigger(rec_finish_client_, "Finish area");
    } else if (c == "rec_cancel") {
      if (rec_goal_) rec_area_client_->async_cancel_goal(rec_goal_);
    } else if (c == "dock_start") {
      recordDock(m.at("name").get<std::string>());
    } else if (c == "dock_cancel") {
      if (dock_goal_) rec_dock_client_->async_cancel_goal(dock_goal_);
    } else {
      send(client, logLine(false, "unknown command " + c));
    }
  }

  bool driveAllowed(std::string & why) const
  {
    if (logic_.is_null() || (now() - logic_time_).seconds() > 3.0) {
      why = "no state from mower_logic";
      return false;
    }
    if (logic_.value("command", "") != "IDLE") {
      why = "mower_logic is busy (" + logic_.value("command", "?") + "), press Stop first";
      return false;
    }
    if (worx_.is_object() && !worx_.value("motors", true)) {
      why = "motors are off (Motors on, in the Mow tab)";
      return false;
    }
    return true;
  }

  void joy(uint64_t client, double v, double w)
  {
    if (!std::isfinite(v) || !std::isfinite(w)) return;
    if (v == 0 && w == 0) {
      if (driving_) stopDriving();
      return;
    }
    std::string why;
    if (!driveAllowed(why)) {
      if (driving_) stopDriving();
      if ((now() - last_refusal_).seconds() > 2.0) {
        last_refusal_ = now();
        send(client, logLine(false, "Driving refused: " + why));
      }
      return;
    }
    publishTwist(std::clamp(v, -1.0, 1.0) * max_linear_, std::clamp(w, -1.0, 1.0) * max_angular_);
    driving_ = true;
    joy_client_ = client;
    last_joy_ = now();
  }

  void stopDriving()
  {
    publishTwist(0, 0);
    driving_ = false;
  }

  void publishTwist(double v, double w)
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = "base_link";
    t.twist.linear.x = v;
    t.twist.angular.z = w;
    joy_pub_->publish(t);
  }

  // ---- manual blade ------------------------------------------------------------

  void blade(uint64_t client, bool on)
  {
    if (!on) {
      setBlade(false);
      return;
    }
    std::string why;
    if (!driveAllowed(why)) {
      event(false, "Blade refused: " + why);
      return;
    }
    blade_page_ = client;
    setBlade(true);
  }

  // Switched on from here, or reported on by the hardware (an extra "off" is harmless).
  bool bladeRunning() const
  {
    return blade_on_ || (worx_.is_object() && worx_.value("blade", Json::object()).value("manual", false));
  }

  void setBlade(bool on)
  {
    if (!on) blade_on_ = false;
    if (!blade_client_ || !blade_client_->service_is_ready()) {
      event(false, std::string("Blade ") + (on ? "on" : "off") + ": /worx/manual_mow not available");
      return;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = on;
    ++blade_requests_;
    blade_client_->async_send_request(req, [this, on](rclcpp::Client<SetBool>::SharedFuture f) {
      --blade_requests_;
      const auto r = f.get();
      if (on && r->success) blade_on_ = true;
      event(r->success, std::string("Blade ") + (on ? "on" : "off") + (r->message.empty() ? "" : ": " + r->message));
    });
  }

  // The hardware switches the blade off on its own (emergency, bump, idle, link); this
  // switches it off when mower_logic leaves IDLE (a mission started some other way).
  void bladeWatch()
  {
    if (!worx_.is_object() || (now() - worx_time_).seconds() > 3.0) return;
    if (!worx_.value("blade", Json::object()).value("manual", false)) return;
    const bool busy = logic_.is_object() && logic_.value("command", "IDLE") != "IDLE";
    if (busy && blade_requests_ == 0 && (now() - last_blade_off_).seconds() > 1.0) {
      last_blade_off_ = now();
      event(false, "mower_logic is busy: manual blade off");
      setBlade(false);
    }
  }

  void setBladePwm(int pwm)
  {
    if (!worx_params_client_->service_is_ready()) {
      event(false, "Blade PWM: /worx_hardware/set_parameters not available");
      return;
    }
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("manual_mow_pwm", pwm).to_parameter_msg());
    worx_params_client_->async_send_request(
      req, [this, pwm](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture f) {
        const auto r = f.get();
        const bool ok = !r->results.empty() && r->results[0].successful;
        event(ok, "Blade PWM " + std::to_string(pwm) + (ok ? "" : ": " + (r->results.empty() ? std::string("no result")
                                                                                             : r->results[0].reason)));
      });
  }

  void trigger(const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & label)
  {
    if (!client->service_is_ready()) {
      event(false, label + ": service not available");
      return;
    }
    client->async_send_request(std::make_shared<Trigger::Request>(),
                               [this, label](rclcpp::Client<Trigger>::SharedFuture f) {
                                 const auto r = f.get();
                                 event(r->success, label + ": " + r->message);
                               });
  }

  void setBool(const rclcpp::Client<SetBool>::SharedPtr & client, bool value, const std::string & label)
  {
    if (!client->service_is_ready()) {
      event(false, label + ": service not available");
      return;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = value;
    client->async_send_request(req, [this, label, value](rclcpp::Client<SetBool>::SharedFuture f) {
      const auto r = f.get();
      event(r->success, label + (value ? " on" : " off") + (r->message.empty() ? "" : ": " + r->message));
    });
  }

  // ---- areas -----------------------------------------------------------------

  std::set<std::string> readDisabled() const
  {
    std::set<std::string> out;
    if (disabled_file_.empty()) return out;
    std::ifstream in(disabled_file_);
    for (std::string line; std::getline(in, line);) {  // same format as mower_logic reads
      line = line.substr(0, line.find('#'));
      const auto b = line.find_first_not_of(" \t\r");
      if (b == std::string::npos) continue;
      out.insert(line.substr(b, line.find_last_not_of(" \t\r") - b + 1));
    }
    return out;
  }

  void setAreaEnabled(const std::string & id, bool enabled)
  {
    if (disabled_file_.empty()) {
      event(false, "No disabled_areas_file configured");
      return;
    }
    auto disabled = readDisabled();
    if (enabled) {
      disabled.erase(id);
    } else {
      disabled.insert(id);
    }
    // Write next to it and rename: mower_logic never reads half a file.
    const std::string tmp = disabled_file_ + ".tmp";
    {
      std::ofstream out(tmp, std::ios::trunc);
      out << "# Areas mower_logic skips when it plans a mission (one id per line). Written by web_ui.\n";
      for (const auto & d : disabled) out << d << "\n";
      if (!out) {
        event(false, "Could not write " + tmp);
        return;
      }
    }
    if (std::rename(tmp.c_str(), disabled_file_.c_str()) != 0) {
      event(false, "Could not replace " + disabled_file_);
      return;
    }
    event(true, "Area " + id + (enabled ? " enabled" : " disabled") + " (used from the next new mission)");
    sendMap(0);
  }

  void removeArea(const std::string & id)
  {
    if (!remove_area_client_->service_is_ready()) {
      event(false, "remove_area: service not available");
      return;
    }
    auto req = std::make_shared<srv::RemoveArea::Request>();
    req->id = id;
    remove_area_client_->async_send_request(req, [this, id](rclcpp::Client<srv::RemoveArea>::SharedFuture f) {
      const auto r = f.get();
      event(r->code == srv::RemoveArea::Response::CODE_SUCCESS, "Remove area " + id + ": " + r->message);
    });
  }

  void sendMap(uint64_t client)
  {
    if (!map_ || !server_ || pages_.empty()) return;
    const auto disabled = readDisabled();
    Json areas = Json::array();
    for (const auto & a : map_->areas) {
      const bool op = a.type == msg::Area::TYPE_OPERATION;
      areas.push_back({{"id", a.id},
                       {"name", a.name},
                       {"type", a.type},
                       {"enabled", !op || (!disabled.count(a.id) && (only_areas_.empty() || only_areas_.count(a.id)))},
                       {"locked", op && !only_areas_.empty() && !only_areas_.count(a.id)},
                       {"pts", points(a.area.polygon, 0.02)}});
    }
    Json docks = Json::array();
    for (const auto & d : map_->docking_stations) {
      docks.push_back({{"id", d.id},
                       {"name", d.name},
                       {"x", r3(d.pose.pose.position.x)},
                       {"y", r3(d.pose.pose.position.y)},
                       {"yaw", r3(yawOf(d.pose.pose.orientation))}});
    }
    server_->send(client, Json{{"t", "map"}, {"areas", areas}, {"docks", docks}}.dump());
  }

  // ---- recording -------------------------------------------------------------

  void recordArea(const std::string & name, int type, bool automatic)
  {
    if (rec_goal_ || rec_pending_) {
      event(false, "An area is already being recorded");
      return;
    }
    if (name.empty() || type < 0 || type > 2) {
      event(false, "Area recording needs a name and a type");
      return;
    }
    if (!rec_area_client_->action_server_is_ready()) {
      event(false, "record_area_boundary: action server not available");
      return;
    }
    RecordArea::Goal goal;
    goal.name = name;
    goal.type = static_cast<uint8_t>(type);
    goal.auto_recording = automatic;
    rclcpp_action::Client<RecordArea>::SendGoalOptions opt;
    opt.goal_response_callback = [this, name](const rclcpp_action::ClientGoalHandle<RecordArea>::SharedPtr & h) {
      rec_pending_ = false;
      rec_goal_ = h;
      event(static_cast<bool>(h), h ? "Recording area " + name : "Area recording refused");
    };
    opt.feedback_callback = [this](rclcpp_action::ClientGoalHandle<RecordArea>::SharedPtr,
                                   const std::shared_ptr<const RecordArea::Feedback> f) {
      rec_status_ = f->message;
      rec_count_ = f->point_count;
      if (!f->area.polygon.points.empty()) {
        rec_points_ = points(f->area.polygon, 0.0);
        rec_dirty_ = true;
      }
    };
    opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<RecordArea>::WrappedResult & r) {
      rec_goal_.reset();
      rec_points_ = Json::array();
      rec_dirty_ = true;
      rec_status_.clear();
      rec_count_ = 0;
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
                      r.result->code == RecordArea::Result::CODE_SUCCESS;
      event(ok, "Area recording: " + (r.result ? r.result->message : std::string("no result")));
    };
    rec_pending_ = true;
    rec_points_ = Json::array();
    rec_area_client_->async_send_goal(goal, opt);
  }

  void recordDock(const std::string & name)
  {
    if (dock_goal_ || dock_pending_) {
      event(false, "The dock is already being recorded");
      return;
    }
    if (!rec_dock_client_->action_server_is_ready()) {
      event(false, "record_docking_station: action server not available");
      return;
    }
    RecordDock::Goal goal;
    goal.name = name.empty() ? "dock" : name;
    rclcpp_action::Client<RecordDock>::SendGoalOptions opt;
    opt.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<RecordDock>::SharedPtr & h) {
      dock_pending_ = false;
      dock_goal_ = h;
      event(static_cast<bool>(h), h ? "Recording the dock: driving forward until the charger is detected"
                                    : "Dock recording refused");
    };
    opt.feedback_callback = [this](rclcpp_action::ClientGoalHandle<RecordDock>::SharedPtr,
                                   const std::shared_ptr<const RecordDock::Feedback> f) { dock_status_ = f->message; };
    opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<RecordDock>::WrappedResult & r) {
      dock_goal_.reset();
      dock_status_.clear();
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
                      r.result->code == RecordDock::Result::CODE_SUCCESS;
      event(ok, "Dock recording: " + (r.result ? r.result->message : std::string("no result")));
    };
    dock_pending_ = true;
    rec_dock_client_->async_send_goal(goal, opt);
  }

  // ---- status ----------------------------------------------------------------

  Json state() const
  {
    const auto t = now();
    std::string why;
    const bool can_drive = driveAllowed(why);
    Json s = {{"t", "state"},
              {"logic", (t - logic_time_).seconds() < 3.0 ? logic_ : Json()},
              {"worx", (t - worx_time_).seconds() < 3.0 ? worx_ : Json()},
              {"gps",
               (t - gps_time_).seconds() < 3.0 ? Json{{"status", gps_status_}, {"acc", r3(gps_acc_)}} : Json()},
              {"drive", {{"ok", can_drive}, {"why", why}, {"max_linear", max_linear_}, {"max_angular", max_angular_}}},
              {"rec", {{"active", static_cast<bool>(rec_goal_)}, {"count", rec_count_}, {"msg", rec_status_}}},
              {"dock_rec", {{"active", static_cast<bool>(dock_goal_)}, {"msg", dock_status_}}},
              {"clients", pages_.size()}};
    return s;
  }

  static std::string logLine(bool ok, const std::string & text)
  {
    return Json{{"t", "log"}, {"ok", ok}, {"msg", text}, {"time", std::time(nullptr)}}.dump();
  }

  // Result of a command: logged, shown on every open page, kept for pages opened later.
  void event(bool ok, const std::string & text)
  {
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
    log_.push_back(logLine(ok, text));
    if (log_.size() > 30) log_.pop_front();
    send(0, log_.back());
  }

  std::unique_ptr<WebServer> server_;
  std::mutex inbox_mutex_;
  std::vector<Inbox> inbox_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t ticks_ = 0;

  std::set<uint64_t> pages_;  // open WebSocket clients
  std::string disabled_file_, pose_topic_;
  std::set<std::string> only_areas_;
  double max_linear_ = 0.3, max_angular_ = 1.0, joy_timeout_ = 0.3;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr joy_pub_;
  bool driving_ = false;
  uint64_t joy_client_ = 0;
  rclcpp::Time last_joy_{0, 0, RCL_ROS_TIME}, last_refusal_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr logic_sub_;
  rclcpp::Subscription<msg::WorxStatus>::SharedPtr worx_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<msg::Map>::SharedPtr map_sub_;
  Json logic_, worx_;
  rclcpp::Time logic_time_{0, 0, RCL_ROS_TIME}, worx_time_{0, 0, RCL_ROS_TIME}, gps_time_{0, 0, RCL_ROS_TIME};
  int gps_status_ = -1;
  double gps_acc_ = 0;
  std::array<double, 3> pose_{};
  bool have_pose_ = false;
  msg::Map::ConstSharedPtr map_;

  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> logic_clients_;
  rclcpp::Client<SetBool>::SharedPtr motors_client_, rec_mode_client_, blade_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr worx_params_client_;
  bool blade_on_ = false;  // switched on from a page (the hardware's state is in worx_)
  uint64_t blade_page_ = 0;
  int blade_requests_ = 0;
  rclcpp::Time last_blade_off_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<Trigger>::SharedPtr rec_point_client_, rec_finish_client_;
  rclcpp::Client<srv::RemoveArea>::SharedPtr remove_area_client_;
  rclcpp_action::Client<RecordArea>::SharedPtr rec_area_client_;
  rclcpp_action::Client<RecordDock>::SharedPtr rec_dock_client_;
  rclcpp_action::ClientGoalHandle<RecordArea>::SharedPtr rec_goal_;
  rclcpp_action::ClientGoalHandle<RecordDock>::SharedPtr dock_goal_;
  bool rec_pending_ = false, dock_pending_ = false, rec_dirty_ = false;
  Json rec_points_ = Json::array();
  std::string rec_status_, dock_status_;
  uint32_t rec_count_ = 0;
  std::deque<std::string> log_;
};

}  // namespace open_mower_next::web_ui

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::web_ui::WebUiNode)

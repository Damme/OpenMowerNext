#include "worx_hardware/worx_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

namespace open_mower_next::worx_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
std::string param(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? def : it->second;
}
double paramD(const hardware_interface::HardwareInfo & info, const std::string & key, double def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? def : std::stod(it->second);
}
bool paramB(const hardware_interface::HardwareInfo & info, const std::string & key, bool def)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return def;
  return it->second == "true" || it->second == "1" || it->second == "True";
}
std::set<std::string> paramSet(const hardware_interface::HardwareInfo & info, const std::string & key,
                               const std::set<std::string> & def)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return def;
  std::set<std::string> out;
  std::stringstream ss(it->second);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
    if (!item.empty()) out.insert(item);
  }
  return out;
}
int toPwm(double v_mps, double pwm_per_mps, int max_pwm)
{
  if (!std::isfinite(v_mps)) return 0;
  // Truncation like the ROS1 node (int assignment).
  const int pwm = static_cast<int>(v_mps * pwm_per_mps);
  return std::clamp(pwm, -max_pwm, max_pwm);
}
}  // namespace

WorxSystem::~WorxSystem()
{
  if (link_) link_->stop();
  stopNode();
}

CallbackReturn WorxSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = info_;
  try {
    cfg_.transport = param(info, "transport", cfg_.transport);
    cfg_.spi_device = param(info, "spi_device", cfg_.spi_device);
    // false: never enable the motors (bench / first contact with a board);
    // the /worx/motors_enabled service can still switch them on.
    motors_enabled_ = paramB(info, "motors_enabled", true);
    cfg_.spi_speed_hz = static_cast<int>(paramD(info, "spi_speed_hz", cfg_.spi_speed_hz));
    cfg_.left_joint = param(info, "left_joint", cfg_.left_joint);
    cfg_.right_joint = param(info, "right_joint", cfg_.right_joint);
    cfg_.mower_joint = param(info, "mower_joint", cfg_.mower_joint);
    cfg_.wheel_radius = paramD(info, "wheel_radius", cfg_.wheel_radius);
    cfg_.wheel_ticks_per_m = paramD(info, "wheel_ticks_per_m", cfg_.wheel_ticks_per_m);
    cfg_.tick_counter_bits = static_cast<int>(paramD(info, "tick_counter_bits", cfg_.tick_counter_bits));
    cfg_.pwm_per_mps = paramD(info, "pwm_per_mps", cfg_.pwm_per_mps);
    cfg_.max_pwm = static_cast<int>(paramD(info, "max_pwm", cfg_.max_pwm));
    cfg_.mow_pwm = static_cast<int>(paramD(info, "mow_pwm", cfg_.mow_pwm));
    cfg_.blade_enabled = paramB(info, "blade_enabled", cfg_.blade_enabled);
    cfg_.manual_mow_pwm = static_cast<int>(paramD(info, "manual_mow_pwm", cfg_.manual_mow_pwm));
    cfg_.manual_mow_max_pwm = static_cast<int>(paramD(info, "manual_mow_max_pwm", cfg_.manual_mow_max_pwm));
    cfg_.charger_from_current = paramB(info, "charger_from_current", cfg_.charger_from_current);
    cfg_.charger_min_ma = static_cast<int>(paramD(info, "charger_min_ma", cfg_.charger_min_ma));
    cfg_.speed_control = paramB(info, "speed_control", cfg_.speed_control);
    cfg_.speed_gains.kp = paramD(info, "speed_kp", cfg_.speed_gains.kp);
    cfg_.speed_gains.ki = paramD(info, "speed_ki", cfg_.speed_gains.ki);
    cfg_.speed_gains.i_max = paramD(info, "speed_i_max", cfg_.speed_gains.i_max);
    cfg_.speed_gains.kv = paramD(info, "speed_kv", cfg_.speed_gains.kv);
    cfg_.speed_gains.ff = paramD(info, "speed_ff", cfg_.speed_gains.ff);
    cfg_.speed_gains.ff_static = paramD(info, "speed_ff_static", cfg_.speed_gains.ff_static);
    cfg_.speed_gains.pos_max = paramD(info, "speed_pos_max", cfg_.speed_gains.pos_max);
    cfg_.speed_filter_tau = paramD(info, "speed_filter_tau", cfg_.speed_filter_tau);
    cfg_.invert_left = paramB(info, "invert_left", cfg_.invert_left);
    cfg_.invert_right = paramB(info, "invert_right", cfg_.invert_right);
    cfg_.link_timeout = paramD(info, "link_timeout", cfg_.link_timeout);
    cfg_.blade_idle_timeout = paramD(info, "blade_idle_timeout", cfg_.blade_idle_timeout);
    cfg_.battery_empty_voltage = paramD(info, "battery_empty_voltage", cfg_.battery_empty_voltage);
    cfg_.battery_full_voltage = paramD(info, "battery_full_voltage", cfg_.battery_full_voltage);
    cfg_.digital_inverted = paramSet(info, "digital_inverted", cfg_.digital_inverted);
    cfg_.log_packets = paramB(info, "log_packets", cfg_.log_packets);
    cfg_.require_crc = paramB(info, "require_crc", cfg_.require_crc);
    cfg_.bump_detection = paramB(info, "bump_detection", cfg_.bump_detection);
    cfg_.dock_wiggle_pwm = static_cast<int>(paramD(info, "dock_wiggle_pwm", cfg_.dock_wiggle_pwm));
    cfg_.dock_wiggles = static_cast<int>(paramD(info, "dock_wiggles", cfg_.dock_wiggles));
    cfg_.bump_min_speed = paramD(info, "bump_min_speed", cfg_.bump_min_speed);
    cfg_.collision_hold = paramD(info, "collision_hold", cfg_.collision_hold);
    cfg_.lift_emergency = paramB(info, "lift_emergency", cfg_.lift_emergency);
    cfg_.resend_period = paramD(info, "resend_period", cfg_.resend_period);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid worx_hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (cfg_.wheel_radius <= 0.0 || cfg_.wheel_ticks_per_m <= 0.0) {
    RCLCPP_ERROR(get_logger(), "wheel_radius and wheel_ticks_per_m must be > 0");
    return CallbackReturn::ERROR;
  }
  // Interfaces are exported after on_init, so check the URDF description.
  auto has_iface = [](const std::vector<hardware_interface::InterfaceInfo> & ifaces, const std::string & name) {
    return std::any_of(ifaces.begin(), ifaces.end(), [&](const auto & i) { return i.name == name; });
  };
  for (const auto & name : {cfg_.left_joint, cfg_.right_joint}) {
    auto it = std::find_if(info.joints.begin(), info.joints.end(), [&](const auto & j) { return j.name == name; });
    if (
      it == info.joints.end() || !has_iface(it->command_interfaces, "velocity") ||
      !has_iface(it->state_interfaces, "position") || !has_iface(it->state_interfaces, "velocity"))
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs a velocity command and position+velocity states", name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (cfg_.manual_mow_max_pwm < 0 || std::abs(cfg_.manual_mow_pwm) > cfg_.manual_mow_max_pwm) {
    RCLCPP_ERROR(get_logger(), "manual_mow_pwm %d outside +-manual_mow_max_pwm %d", cfg_.manual_mow_pwm,
                 cfg_.manual_mow_max_pwm);
    return CallbackReturn::ERROR;
  }
  manual_mow_pwm_ = cfg_.manual_mow_pwm;
  odo_left_ = WheelOdometer(cfg_.wheel_ticks_per_m, cfg_.tick_counter_bits);
  odo_right_ = WheelOdometer(cfg_.wheel_ticks_per_m, cfg_.tick_counter_bits);
  RCLCPP_INFO(
    get_logger(), "Worx hardware: transport=%s device=%s ticks/m=%.1f pwm/mps=%.0f max_pwm=%d mow_pwm=%d",
    cfg_.transport.c_str(), cfg_.spi_device.c_str(), cfg_.wheel_ticks_per_m, cfg_.pwm_per_mps,
    cfg_.max_pwm, cfg_.mow_pwm);
  if (!cfg_.blade_enabled) RCLCPP_WARN(get_logger(), "Blade disabled (blade_enabled=false): blade PWM stays 0");
  gains_ = cfg_.speed_gains;
  RCLCPP_INFO(
    get_logger(), "Wheel distance control %s (ff=%.0f+%.0f*v kp=%.0f ki=%.0f kv=%.0f i_max=%.0f pos_max=%.3f), charger from %s",
    cfg_.speed_control ? "on" : "off", gains_.ff_static, gains_.ff, gains_.kp, gains_.ki, gains_.kv, gains_.i_max,
    gains_.pos_max,
    cfg_.charger_from_current ? "charge current" : "InCharger");
  return CallbackReturn::SUCCESS;
}

void WorxSystem::startNode()
{
  if (node_) return;
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);  // don't pick up the controller manager's __node remap
  node_ = std::make_shared<rclcpp::Node>("worx_hardware", opts);
  battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("/power", 10);
  charger_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/power/charger_present", 10);
  status_pub_ = node_->create_publisher<open_mower_next::msg::WorxStatus>("/worx/status", 10);
  emergency_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "/worx/emergency", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                              std_srvs::srv::SetBool::Response::SharedPtr res) {
      emergency_ = req->data;
      blade_lockout_ = true;
      if (req->data) {
        RCLCPP_ERROR(node_->get_logger(), "Emergency set: motors stopped");
        sendSpeed(0, 0, 0, true);
        link_->send(cmdMotorsDisable(), true);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Emergency cleared");
        {
          std::lock_guard<std::mutex> lock(mutex_);
          emergency_cleared_at_ = std::chrono::steady_clock::now();
        }
        link_->send(cmdResetEmergency(), true);  // older firmware: "Unknown command", harmless
        if (active_ && motors_enabled_) link_->send(cmdMotorsEnable());
      }
      res->success = true;
      res->message = req->data ? "emergency latched" : "emergency cleared";
    });
  motors_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "/worx/motors_enabled", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr res) {
      motors_enabled_ = req->data;
      blade_lockout_ = true;
      if (!req->data) sendSpeed(0, 0, 0, true);
      link_->send(req->data ? cmdMotorsEnable() : cmdMotorsDisable(), true);
      res->success = true;
    });
  manual_mow_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "/worx/manual_mow", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                               std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (!req->data) {
        if (manual_mow_.exchange(false)) RCLCPP_INFO(node_->get_logger(), "Manual blade off");
        blade_lockout_ = true;  // released again once the mower joint's effort is 0
        res->success = true;
        return;
      }
      std::string why;
      if (manualMowRefused(why)) {
        res->success = false;
        res->message = why;
        return;
      }
      // Lockout first: write() switches manual mowing off while the blade is locked out.
      blade_lockout_ = false;
      manual_mow_ = true;
      RCLCPP_WARN(node_->get_logger(), "Manual blade on: PWM %d", manual_mow_pwm_.load());
      res->success = true;
      res->message = "PWM " + std::to_string(manual_mow_pwm_.load());
    });
  docking_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "/worx/docking_mode", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                 std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (docking_mode_ != req->data) RCLCPP_INFO(node_->get_logger(), "Docking mode %s", req->data ? "on" : "off");
      docking_mode_ = req->data;
      if (req->data) collision_ = false;  // a bump latched on the way in must not block the approach
      dock_wiggles_ = 0;
      res->success = true;
    });
  if (fake_board_) {
    // Simulation hook: lets a sim node put the emulated board "in the charger".
    fake_charger_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/worx/fake/set_in_charger", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                          std_srvs::srv::SetBool::Response::SharedPtr res) {
        fake_board_->setInCharger(req->data);
        res->success = true;
      });
    fake_collision_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/worx/fake/set_collision", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                         std_srvs::srv::SetBool::Response::SharedPtr res) {
        fake_board_->setCollision(req->data);
        res->success = true;
      });
    fake_lift_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/worx/fake/set_lift", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                    std_srvs::srv::SetBool::Response::SharedPtr res) {
        fake_board_->setLift(req->data);
        res->success = true;
      });
    fake_speed_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/worx/fake/speed_factor", 10, [this](std_msgs::msg::Float64::ConstSharedPtr m) { fake_board_->setSpeedFactor(m->data); });
    fake_battery_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/worx/fake/battery_mv", 10, [this](std_msgs::msg::Int32::ConstSharedPtr m) { fake_board_->setBatteryMv(m->data); });
  }
  wheel_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/worx/wheel_scale", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::Float64::ConstSharedPtr m) {
      if (!std::isfinite(m->data) || m->data < 0.8 || m->data > 1.2) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring wheel scale %.4f (outside 0.8..1.2)", m->data);
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (m->data != wheel_scale_) RCLCPP_INFO(node_->get_logger(), "Wheel scale %.4f -> %.4f", wheel_scale_, m->data);
      wheel_scale_ = m->data;
    });
  // Speed loop tuning at runtime: ros2 param set /worx_hardware speed_kp 2000
  node_->declare_parameter("speed_kp", gains_.kp);
  node_->declare_parameter("speed_ki", gains_.ki);
  node_->declare_parameter("speed_i_max", gains_.i_max);
  node_->declare_parameter("speed_kv", gains_.kv);
  node_->declare_parameter("speed_ff", gains_.ff);
  node_->declare_parameter("speed_ff_static", gains_.ff_static);
  node_->declare_parameter("speed_pos_max", gains_.pos_max);
  // Manual blade PWM, sign = direction (of the commanded blade too):
  // ros2 param set /worx_hardware manual_mow_pwm -1500
  node_->declare_parameter("manual_mow_pwm", manual_mow_pwm_.load());
  gains_cb_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    for (const auto & p : ps) {
      if (p.get_name() != "manual_mow_pwm") continue;
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        r.successful = false;
        r.reason = "manual_mow_pwm must be an integer";
        return r;
      }
      const auto v = p.as_int();
      if (std::abs(v) > cfg_.manual_mow_max_pwm) {
        r.successful = false;
        r.reason = "manual_mow_pwm must be within +-" + std::to_string(cfg_.manual_mow_max_pwm);
        return r;
      }
      // Reversing a spinning blade at full PWM: switch it off first.
      if (manual_mow_ && (v == 0 || (v > 0) != (manual_mow_pwm_ > 0))) {
        r.successful = false;
        r.reason = "switch the manual blade off before changing its direction";
        return r;
      }
      if (v != manual_mow_pwm_) RCLCPP_INFO(node_->get_logger(), "Manual blade PWM %d", static_cast<int>(v));
      manual_mow_pwm_ = static_cast<int>(v);
    }
    std::lock_guard<std::mutex> lock(gains_mutex_);
    bool gains = false;
    for (const auto & p : ps) {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) continue;
      if (p.get_name() == "speed_kp") gains_.kp = p.as_double();
      if (p.get_name() == "speed_ki") gains_.ki = p.as_double();
      if (p.get_name() == "speed_i_max") gains_.i_max = p.as_double();
      if (p.get_name() == "speed_kv") gains_.kv = p.as_double();
      if (p.get_name() == "speed_ff") gains_.ff = p.as_double();
      if (p.get_name() == "speed_ff_static") gains_.ff_static = p.as_double();
      if (p.get_name() == "speed_pos_max") gains_.pos_max = p.as_double();
      gains = true;
    }
    if (!gains) return r;
    RCLCPP_INFO(node_->get_logger(), "Speed gains: ff=%.0f+%.0f*v kp=%.0f ki=%.0f kv=%.0f i_max=%.0f pos_max=%.3f",
      gains_.ff_static, gains_.ff, gains_.kp, gains_.ki, gains_.kv, gains_.i_max, gains_.pos_max);
    return r;
  });
  status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() { publishStatus(); });
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
}

void WorxSystem::stopNode()
{
  if (executor_) executor_->cancel();
  if (executor_thread_.joinable()) executor_thread_.join();
  status_timer_.reset();
  gains_cb_.reset();
  emergency_srv_.reset();
  motors_srv_.reset();
  docking_srv_.reset();
  manual_mow_srv_.reset();
  fake_charger_srv_.reset();
  fake_speed_sub_.reset();
  fake_collision_srv_.reset();
  fake_lift_srv_.reset();
  fake_battery_sub_.reset();
  executor_.reset();
  node_.reset();
}

CallbackReturn WorxSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::unique_ptr<Transport> transport;
  if (cfg_.transport == "fake") {
    FakeBoardTransport::Options o;
    o.ticks_per_m = cfg_.wheel_ticks_per_m;
    o.pwm_per_mps = cfg_.pwm_per_mps;
    auto fake = std::make_unique<FakeBoardTransport>(o);
    fake_board_ = fake.get();
    transport = std::move(fake);
  } else if (cfg_.transport == "spidev") {
    transport = std::make_unique<SpidevTransport>(cfg_.spi_device, static_cast<uint32_t>(cfg_.spi_speed_hz));
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown transport '%s' (spidev|fake)", cfg_.transport.c_str());
    return CallbackReturn::ERROR;
  }
  WorxLink::Options link_opts;
  link_opts.require_crc = cfg_.require_crc;
  link_ = std::make_unique<WorxLink>(std::move(transport), link_opts);
  std::string error;
  if (!link_->start([this](const std::string & m) { onBoardMessage(m); }, error)) {
    RCLCPP_ERROR(get_logger(), "Worx link: %s", error.c_str());
    link_.reset();
    return CallbackReturn::ERROR;
  }
  startNode();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WorxSystem::on_activate(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    set_state(cfg_.left_joint + "/position", dist_left_ / cfg_.wheel_radius);
    set_state(cfg_.right_joint + "/position", dist_right_ / cfg_.wheel_radius);
  }
  set_state(cfg_.left_joint + "/velocity", 0.0);
  set_state(cfg_.right_joint + "/velocity", 0.0);
  set_command(cfg_.left_joint + "/velocity", 0.0);
  set_command(cfg_.right_joint + "/velocity", 0.0);
  if (has_command(cfg_.mower_joint + "/effort")) set_command(cfg_.mower_joint + "/effort", 0.0);
  sendSpeed(0, 0, 0, true);
  if (motors_enabled_ && !emergency_) link_->send(cmdMotorsEnable());
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn WorxSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  blade_lockout_ = true;
  manual_mow_ = false;
  if (link_) {
    sendSpeed(0, 0, 0, true);
    link_->send(cmdMotorsDisable());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn WorxSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stopNode();
  if (link_) {
    link_->stop();
    link_.reset();
  }
  fake_board_ = nullptr;
  return CallbackReturn::SUCCESS;
}

CallbackReturn WorxSystem::on_shutdown(const rclcpp_lifecycle::State & s)
{
  if (link_) {
    sendSpeed(0, 0, 0, true);
    link_->send(cmdMotorsDisable());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // let the link send them
  }
  return on_cleanup(s);
}

void WorxSystem::sendSpeed(int left, int right, int mow, bool force)
{
  if (!link_) return;
  const auto now = std::chrono::steady_clock::now();
  // Unchanged commands are repeated every resend_period: the firmware answers a
  // forward command while BlockForward is latched with 0 PWM (and clears the
  // latch), so an identical follow-up must reach it again.
  if (
    !force && sent_once_ && left == last_pwm_l_ && right == last_pwm_r_ && mow == last_pwm_mow_ &&
    std::chrono::duration<double>(now - last_send_).count() < cfg_.resend_period)
  {
    return;
  }
  last_send_ = now;
  last_pwm_l_ = left;
  last_pwm_r_ = right;
  last_pwm_mow_ = mow;
  sent_once_ = true;
  link_->send(cmdSetSpeed(left, right, mow), /*urgent=*/true);
}

return_type WorxSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool fresh = std::chrono::steady_clock::now() - last_pulse_ < std::chrono::milliseconds(500);
  const double sl = cfg_.invert_left ? -1.0 : 1.0;
  const double sr = cfg_.invert_right ? -1.0 : 1.0;
  set_state(cfg_.left_joint + "/position", sl * dist_left_ / cfg_.wheel_radius);
  set_state(cfg_.right_joint + "/position", sr * dist_right_ / cfg_.wheel_radius);
  set_state(cfg_.left_joint + "/velocity", fresh ? sl * vel_left_ / cfg_.wheel_radius : 0.0);
  set_state(cfg_.right_joint + "/velocity", fresh ? sr * vel_right_ / cfg_.wheel_radius : 0.0);
  if (has_state(cfg_.mower_joint + "/position")) {
    set_state(cfg_.mower_joint + "/position", 0.0);  // the board reports no blade angle
  }
  if (has_state(cfg_.mower_joint + "/velocity")) {
    const int pulses = last_.motor_pulse ? last_.motor_pulse->mow : 0;
    set_state(cfg_.mower_joint + "/velocity", fresh ? static_cast<double>(pulses) : 0.0);
  }
  return return_type::OK;
}

return_type WorxSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_ || !link_) return return_type::OK;
  double wl = get_command<double>(cfg_.left_joint + "/velocity");
  double wr = get_command<double>(cfg_.right_joint + "/velocity");
  double blade = has_command(cfg_.mower_joint + "/effort") ? get_command<double>(cfg_.mower_joint + "/effort") : 0.0;
  if (!std::isfinite(wl)) wl = 0.0;
  if (!std::isfinite(wr)) wr = 0.0;
  if (!std::isfinite(blade)) blade = 0.0;
  const bool manual = manual_mow_;
  if (manual) blade = 1.0;  // same idle timeout and lockouts as a commanded blade
  if (cfg_.invert_left) wl = -wl;
  if (cfg_.invert_right) wr = -wr;

  const bool link_ok = link_->secondsSinceRx() < cfg_.link_timeout;
  int pl, pr;
  if (cfg_.speed_control && link_ok && motors_enabled_ && !emergency_) {
    // Wheel travel from the ticks, carried forward from the last 20 Hz report
    // with the filtered speed (at most 0.1 s) so the tracked distance doesn't
    // saw-tooth between reports.
    double ml, mr, xl, xr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto age = std::chrono::steady_clock::now() - last_pulse_;
      const bool fresh = age < std::chrono::milliseconds(500);
      ml = fresh ? filt_left_.value() : 0.0;
      mr = fresh ? filt_right_.value() : 0.0;
      const double ahead = std::min(std::chrono::duration<double>(age).count(), 0.1);
      xl = dist_left_ + ml * ahead;
      xr = dist_right_ + mr * ahead;
    }
    SpeedGains g;
    {
      std::lock_guard<std::mutex> lock(gains_mutex_);
      g = gains_;
    }
    const double dt = period.seconds();
    pl = pi_left_.update(wl * cfg_.wheel_radius, xl, ml, dt, g, cfg_.max_pwm);
    pr = pi_right_.update(wr * cfg_.wheel_radius, xr, mr, dt, g, cfg_.max_pwm);
  } else {
    pi_left_.reset();
    pi_right_.reset();
    pl = toPwm(wl * cfg_.wheel_radius, cfg_.pwm_per_mps, cfg_.max_pwm);
    pr = toPwm(wr * cfg_.wheel_radius, cfg_.pwm_per_mps, cfg_.max_pwm);
  }
  const auto now = std::chrono::steady_clock::now();
  // Idle time only counts while the blade is requested and the wheels are commanded still.
  if (pl != 0 || pr != 0 || blade <= 0.0) last_motion_cmd_ = now;
  const bool idle = cfg_.blade_idle_timeout > 0 &&
    std::chrono::duration<double>(now - last_motion_cmd_).count() > cfg_.blade_idle_timeout;
  if (emergency_ || !motors_enabled_ || !link_ok || idle) {
    if (idle && !blade_lockout_ && blade > 0.0) {
      RCLCPP_WARN(get_logger(), "No drive commands for %.0f s: blade off", cfg_.blade_idle_timeout);
    }
    blade_lockout_ = true;
  } else if (blade <= 0.0 && blade_lockout_) {
    blade_lockout_ = false;  // blade was commanded off: it may start again on the next command
  }
  if (collision_) {
    // Bumped: no forward wheel motion (as the firmware), reversing/turning back is fine.
    const bool forward = pl > 0 || pr > 0;
    if (pl > 0) pi_left_.reset();
    if (pr > 0) pi_right_.reset();
    pl = std::min(pl, 0);
    pr = std::min(pr, 0);
    blade_lockout_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!forward && std::chrono::duration<double>(now - last_bump_).count() > cfg_.collision_hold) {
      collision_ = false;
      RCLCPP_INFO(get_logger(), "Bump cleared");
    }
  }
  if (docking_mode_ && (pl > 0 || pr > 0)) {
    // Firmware BlockForward (bumper on the dock): it clears as soon as one wheel is
    // commanded backwards (motorctrl.c). Daniel: wiggle - one wheel briefly back,
    // alternating sides, then push on; seats the robot on the contacts.
    bool fw_blocked = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fw_blocked = last_.motor_pulse && last_.motor_pulse->block_forward && *last_.motor_pulse->block_forward == 1;
    }
    // The firmware re-arms BlockForward on every sensor tick while the bumper stays
    // pressed: at most one wiggle per 0.4 s (forward in between) and dock_wiggles per
    // docking mode, so the wiggling can't walk the robot back out of the dock.
    if (fw_blocked && now >= dock_next_wiggle_ && dock_wiggles_ < cfg_.dock_wiggles) {
      dock_clear_until_ = now + std::chrono::milliseconds(60);
      dock_next_wiggle_ = now + std::chrono::milliseconds(400);
      dock_wiggle_left_ = !dock_wiggle_left_;
      ++dock_wiggles_;
    }
    if (now < dock_clear_until_) {
      pl = dock_wiggle_left_ ? -cfg_.dock_wiggle_pwm : 0;
      pr = dock_wiggle_left_ ? 0 : -cfg_.dock_wiggle_pwm;
      pi_left_.reset();
      pi_right_.reset();
    }
  }
  // The commanded (mission) blade turns the way manual_mow_pwm's sign says. The direction
  // is taken while the blade isn't driven, so a change only applies on its next start.
  if (last_pwm_mow_ == 0) {
    const int dir = manual_mow_pwm_ < 0 ? -1 : 1;
    if (dir != mow_dir_) RCLCPP_INFO(get_logger(), "Blade direction %s", dir < 0 ? "reverse" : "forward");
    mow_dir_ = dir;
  }
  int pm = blade_lockout_ || !cfg_.blade_enabled
    ? 0 : mow_dir_ * static_cast<int>(std::clamp(blade, 0.0, 1.0) * cfg_.mow_pwm);
  if (manual) {
    if (blade_lockout_) {
      // Emergency, motors off, link loss, bump or idle: stays off until switched on again.
      if (manual_mow_.exchange(false)) RCLCPP_WARN(get_logger(), "Manual blade switched off (safety stop)");
    } else if (cfg_.blade_enabled) {
      pm = manual_mow_pwm_;
    }
  }
  if (emergency_ || !motors_enabled_ || !link_ok) {
    pl = pr = pm = 0;
  }
  sendSpeed(pl, pr, pm, false);
  return return_type::OK;
}

void WorxSystem::onBoardMessage(const std::string & msg)
{
  const auto m = parseMessage(msg);

  if (cfg_.log_packets && node_) {
    // One line per message type every 2 s (ROS1 worx_comms logged the same way).
    static std::map<std::string, std::chrono::steady_clock::time_point> last_log;
    const std::string key = m.is_json ? msg.substr(0, std::min<size_t>(msg.find(':'), 24)) : msg.substr(0, 5);
    const auto now = std::chrono::steady_clock::now();
    auto & t = last_log[key];
    if (now - t > std::chrono::seconds(2) || m.motor_state || m.power_state) {
      t = now;
      if (!m.is_json && msg.rfind("DEBUG", 0) == 0) {
        RCLCPP_WARN(node_->get_logger(), "!!! %s", msg.c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), ">>> %s", msg.c_str());
      }
    }
  }
  if (!m.is_json) return;

  std::lock_guard<std::mutex> lock(mutex_);
  if (m.motor_pulse) {
    const auto now = std::chrono::steady_clock::now();
    // BlockForward while driving forward = bump. It also reads 1 whenever the
    // robot stands still, so it only counts if the robot moved within the last
    // 0.3 s. (Not "moving in this sample": the firmware clears the latch on an
    // all-zero command and re-arms it 1 ms later, so the first reports can show 0
    // while the wheels already stop.)
    if (0.5 * (vel_left_ + vel_right_) > cfg_.bump_min_speed) last_moving_ = now;
    const bool blocked = m.motor_pulse->block_forward && *m.motor_pulse->block_forward == 1;
    const bool moved_recently = now - last_moving_ < std::chrono::milliseconds(300);
    if (cfg_.bump_detection && !docking_mode_ && blocked && moved_recently && last_pwm_l_ > 0 && last_pwm_r_ > 0) {
      registerBump("BlockForward");
    }
    // Bumper presses counted by the firmware (5 ms debounce): none is missed
    // between reports. Same rule as a Collision in Digital.
    if (m.motor_pulse->bumps) {
      const uint32_t b = *m.motor_pulse->bumps;
      if (
        last_board_bumps_ && b > *last_board_bumps_ && cfg_.bump_detection && !docking_mode_ &&
        (collision_ || (last_pwm_l_ > 0 && last_pwm_r_ > 0)))
      {
        registerBump("Bumper");
      }
      last_board_bumps_ = b;  // a board reset (smaller count) just re-baselines
    }
    if (m.motor_pulse->emergency && *m.motor_pulse->emergency == 1 && !emergency_ &&
        now - emergency_cleared_at_ > std::chrono::milliseconds(1500))
    {
      const int reason = m.motor_pulse->emergency_reason.value_or(0);
      RCLCPP_ERROR(
        get_logger(), "Board emergency (%s): latched (clear with /worx/emergency false)",
        reason ? emergencyReasonText(reason).c_str() : "reason unknown");
      emergency_ = true;
      blade_lockout_ = true;
    }
    const double dl = wheel_scale_ * odo_left_.update(m.motor_pulse->left, m.motor_pulse->dir_left);
    const double dr = wheel_scale_ * odo_right_.update(m.motor_pulse->right, m.motor_pulse->dir_right);
    dist_left_ += dl;
    dist_right_ += dr;
    // Speed over the board's own sample interval when it reports one ("ms"):
    // frames that arrive bunched up (SPI pauses) gave speed spikes like 2.07 m/s.
    double dt = std::chrono::duration<double>(now - last_pulse_).count();
    if (m.motor_pulse->ms) {
      if (last_board_ms_) {
        const double board_dt = static_cast<uint32_t>(*m.motor_pulse->ms - *last_board_ms_) / 1000.0;
        if (board_dt > 1e-3 && board_dt < 1.0) dt = board_dt;
      }
      last_board_ms_ = m.motor_pulse->ms;
    }
    if (dt > 1e-3 && dt < 1.0) {
      vel_left_ = dl / dt;
      vel_right_ = dr / dt;
      filt_left_.update(vel_left_, dt, cfg_.speed_filter_tau);
      filt_right_.update(vel_right_, dt, cfg_.speed_filter_tau);
    } else {
      vel_left_ = vel_right_ = 0.0;
      filt_left_.reset();
      filt_right_.reset();
    }
    last_pulse_ = now;
    last_.motor_pulse = m.motor_pulse;
  }
  if (m.battery) {
    battery_ = m.battery;
    if (m.battery->state) last_.power_state = m.battery->state;  // repeated every 1.25 s
  }
  if (m.motor_current) last_.motor_current = m.motor_current;
  if (m.motor_pwm) last_.motor_pwm = m.motor_pwm;
  if (m.digital_corrupt) RCLCPP_WARN(get_logger(), "Garbled Digital frame ignored: %s", m.digital_corrupt->c_str());
  if (m.digital) {
    last_.digital = m.digital;
    auto active = [&](const std::string & name) {
      const auto it = m.digital->find(name);
      if (it == m.digital->end()) return false;
      return cfg_.digital_inverted.count(name) > 0 ? it->second == 0 : it->second != 0;
    };
    // Digital is up to 1.25 s old: it only starts a bump while driving forward
    // (a stale report during the back-up must not count again).
    // At the dock the bumper is pressed on purpose (docking mode): no bump.
    if (cfg_.bump_detection && !docking_mode_ && active("Collision") && (collision_ || (last_pwm_l_ > 0 && last_pwm_r_ > 0))) {
      registerBump("Collision");
    }
    // The Lift input is the shell moving up against the chassis (bump, charging
    // contacts, someone lifting the shell), not the robot leaving the ground: a
    // bump, blade off (the firmware also cuts the blade itself). Not while docking
    // or charging, where the contacts push the shell up. lift_emergency: true
    // restores the old latched emergency; tilt is the firmware's real emergency.
    const bool charging = battery_ && battery_->ma >= cfg_.charger_min_ma;
    const bool lift = active("Lift") && !docking_mode_ && !charging;
    if (lift && !lift_) {
      blade_lockout_ = true;
      if (cfg_.lift_emergency && !emergency_) {
        RCLCPP_ERROR(get_logger(), "Lift detected: emergency latched (clear with /worx/emergency false)");
        emergency_ = true;
      } else if (cfg_.bump_detection) {
        registerBump("Lift");
      }
    }
    lift_ = lift;
  }
  if (m.analog) last_.analog = m.analog;
  if (m.link) last_.link = m.link;
  if (m.boundary) last_.boundary = m.boundary;
  if (m.motor_state) last_.motor_state = m.motor_state;
  if (m.power_state) last_.power_state = m.power_state;
}

void WorxSystem::registerBump(const char * source)
{
  last_bump_ = std::chrono::steady_clock::now();
  blade_lockout_ = true;
  if (!collision_.exchange(true)) {
    ++bumps_;
    RCLCPP_WARN(get_logger(), "Bump (%s): forward blocked, blade off", source);
  }
}

bool WorxSystem::manualMowRefused(std::string & why) const
{
  if (!active_ || !link_) {
    why = "hardware not active";
  } else if (!cfg_.blade_enabled) {
    why = "blade disabled (blade_enabled=false)";
  } else if (emergency_) {
    why = "emergency latched";
  } else if (!motors_enabled_) {
    why = "motors are off";
  } else if (link_->secondsSinceRx() >= cfg_.link_timeout) {
    why = "no board link";
  } else if (collision_) {
    why = "bump latched";
  } else if (manual_mow_pwm_ == 0) {
    why = "manual_mow_pwm is 0";
  }
  return !why.empty();
}

void WorxSystem::publishStatus()
{
  if (!link_ || !node_) return;
  open_mower_next::msg::WorxStatus st;
  sensor_msgs::msg::BatteryState bat;
  std::optional<bool> charger;
  std::string motor_state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    st.header.stamp = node_->now();
    st.link_ok = link_->secondsSinceRx() < cfg_.link_timeout;
    st.emergency = emergency_;
    st.motors_enabled = motors_enabled_;
    st.collision = collision_;
    st.lift = lift_;
    st.bumps = bumps_;
    st.motor_state = motor_state = last_.motor_state.value_or("");
    st.power_state = last_.power_state.value_or("");
    st.left_pwm_cmd = last_pwm_l_;
    st.right_pwm_cmd = last_pwm_r_;
    st.mow_pwm_cmd = last_pwm_mow_;
    st.manual_mow = manual_mow_;
    st.manual_mow_pwm = manual_mow_pwm_;
    st.manual_mow_max_pwm = cfg_.manual_mow_max_pwm;
    if (last_.motor_pwm) st.motor_pwm = {last_.motor_pwm->left, last_.motor_pwm->right, last_.motor_pwm->mow};
    if (last_.motor_current)
      st.motor_current = {last_.motor_current->left, last_.motor_current->right, last_.motor_current->mow};
    st.mow_pulses = last_.motor_pulse ? last_.motor_pulse->mow : 0;
    st.board_emergency = last_.motor_pulse && last_.motor_pulse->emergency ? *last_.motor_pulse->emergency : -1;
    st.block_forward = last_.motor_pulse && last_.motor_pulse->block_forward ? *last_.motor_pulse->block_forward : -1;
    if (last_.digital) {
      for (const auto & [name, raw] : *last_.digital) {
        st.digital_names.push_back(name);
        st.digital_raw.push_back(raw);
        const bool inverted = cfg_.digital_inverted.count(name) > 0;
        st.digital_active.push_back(inverted ? raw == 0 : raw != 0);
      }
    }
    if (last_.analog) {
      for (const auto & [name, v] : *last_.analog) {
        st.analog_names.push_back(name);
        st.analog_values.push_back(v);
      }
    }
    if (battery_ && battery_->contact) {  // charger pins (firmware 2026-09+)
      st.analog_names.push_back("ChargerConnected");
      st.analog_values.push_back(*battery_->contact);
    }
    if (battery_ && battery_->charge_enable) {
      st.analog_names.push_back("ChargerEnable");
      st.analog_values.push_back(*battery_->charge_enable);
    }
    if (last_.link) {  // board's view of the SPI link (firmware 2026-09-30+)
      for (const auto & [name, v] : *last_.link) {
        st.analog_names.push_back("Link." + name);
        st.analog_values.push_back(v);
      }
    }
    st.analog_names.push_back("Link.PiCrcErr");
    st.analog_values.push_back(static_cast<int>(link_->crcErrors()));
    st.analog_names.push_back("Link.PiNoCrc");
    st.analog_values.push_back(static_cast<int>(link_->rxWithoutCrc()));
    if (last_.motor_pulse && last_.motor_pulse->blade_lock) {
      st.analog_names.push_back("BladeLock");
      st.analog_values.push_back(*last_.motor_pulse->blade_lock);
    }
    if (last_.motor_pulse && last_.motor_pulse->emergency_reason) {
      st.analog_names.push_back("EmgReason");
      st.analog_values.push_back(*last_.motor_pulse->emergency_reason);
    }
    if (last_.boundary) {
      for (const auto & [name, v] : *last_.boundary) {
        st.boundary_names.push_back(name);
        st.boundary_values.push_back(v);
      }
    }
    st.rx_messages = link_->rxMessages();
    st.rx_dropped = link_->rxDropped();
    st.tick_glitches = odo_left_.glitches() + odo_right_.glitches();

    if (battery_) {
      const auto & b = *battery_;
      st.battery_voltage = static_cast<float>(b.mv / 1000.0);
      st.battery_current = static_cast<float>(b.ma / 1000.0);
      st.battery_temperature = static_cast<float>(b.temp_raw / 10.0);
      st.in_charger = cfg_.charger_from_current || !b.in_charger ? b.ma >= cfg_.charger_min_ma : *b.in_charger != 0;
      charger = st.in_charger;

      bat.header.stamp = st.header.stamp;
      bat.voltage = st.battery_voltage;
      bat.current = st.battery_current;
      bat.temperature = st.battery_temperature;
      bat.charge = bat.capacity = bat.design_capacity = std::nanf("");
      const double span = cfg_.battery_full_voltage - cfg_.battery_empty_voltage;
      bat.percentage = span > 0 ? static_cast<float>(std::clamp((st.battery_voltage - cfg_.battery_empty_voltage) / span, 0.0, 1.0)) : std::nanf("");
      bat.power_supply_status = st.in_charger
        ? (b.ma > 0 ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                    : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING)
        : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
      bat.present = true;
    }
  }
  status_pub_->publish(st);
  if (charger) {
    battery_pub_->publish(bat);
    std_msgs::msg::Bool c;
    c.data = *charger;
    charger_pub_->publish(c);
  }

  // The firmware drops to DISABLE/IDLE on its own (watchdog, sensors); re-enable
  // while we are active, like worx_comms did for autonomous/recording states.
  static auto last_enable = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (
    active_ && motors_enabled_ && !emergency_ && st.link_ok &&
    (motor_state == "MOTORREQ_DISABLE" || motor_state == "MOTORREQ_IDLE") &&
    now - last_enable > std::chrono::seconds(1))
  {
    last_enable = now;
    link_->send(cmdMotorsEnable());
  }
}

}  // namespace open_mower_next::worx_hardware

PLUGINLIB_EXPORT_CLASS(open_mower_next::worx_hardware::WorxSystem, hardware_interface::SystemInterface)

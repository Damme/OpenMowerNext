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
    cfg_.invert_left = paramB(info, "invert_left", cfg_.invert_left);
    cfg_.invert_right = paramB(info, "invert_right", cfg_.invert_right);
    cfg_.link_timeout = paramD(info, "link_timeout", cfg_.link_timeout);
    cfg_.blade_idle_timeout = paramD(info, "blade_idle_timeout", cfg_.blade_idle_timeout);
    cfg_.battery_empty_voltage = paramD(info, "battery_empty_voltage", cfg_.battery_empty_voltage);
    cfg_.battery_full_voltage = paramD(info, "battery_full_voltage", cfg_.battery_full_voltage);
    cfg_.digital_inverted = paramSet(info, "digital_inverted", cfg_.digital_inverted);
    cfg_.log_packets = paramB(info, "log_packets", cfg_.log_packets);
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
  odo_left_ = WheelOdometer(cfg_.wheel_ticks_per_m, cfg_.tick_counter_bits);
  odo_right_ = WheelOdometer(cfg_.wheel_ticks_per_m, cfg_.tick_counter_bits);
  RCLCPP_INFO(
    get_logger(), "Worx hardware: transport=%s device=%s ticks/m=%.1f pwm/mps=%.0f max_pwm=%d mow_pwm=%d",
    cfg_.transport.c_str(), cfg_.spi_device.c_str(), cfg_.wheel_ticks_per_m, cfg_.pwm_per_mps,
    cfg_.max_pwm, cfg_.mow_pwm);
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
  if (fake_board_) {
    // Simulation hook: lets a sim node put the emulated board "in the charger".
    fake_charger_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/worx/fake/set_in_charger", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                          std_srvs::srv::SetBool::Response::SharedPtr res) {
        fake_board_->setInCharger(req->data);
        res->success = true;
      });
  }
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
  emergency_srv_.reset();
  motors_srv_.reset();
  fake_charger_srv_.reset();
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
  link_ = std::make_unique<WorxLink>(std::move(transport), WorxLink::Options{});
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
    set_state(cfg_.left_joint + "/position", odo_left_.distance() / cfg_.wheel_radius);
    set_state(cfg_.right_joint + "/position", odo_right_.distance() / cfg_.wheel_radius);
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
  if (!force && sent_once_ && left == last_pwm_l_ && right == last_pwm_r_ && mow == last_pwm_mow_) return;
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
  set_state(cfg_.left_joint + "/position", sl * odo_left_.distance() / cfg_.wheel_radius);
  set_state(cfg_.right_joint + "/position", sr * odo_right_.distance() / cfg_.wheel_radius);
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

return_type WorxSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !link_) return return_type::OK;
  double wl = get_command<double>(cfg_.left_joint + "/velocity");
  double wr = get_command<double>(cfg_.right_joint + "/velocity");
  double blade = has_command(cfg_.mower_joint + "/effort") ? get_command<double>(cfg_.mower_joint + "/effort") : 0.0;
  if (!std::isfinite(wl)) wl = 0.0;
  if (!std::isfinite(wr)) wr = 0.0;
  if (!std::isfinite(blade)) blade = 0.0;
  if (cfg_.invert_left) wl = -wl;
  if (cfg_.invert_right) wr = -wr;

  int pl = toPwm(wl * cfg_.wheel_radius, cfg_.pwm_per_mps, cfg_.max_pwm);
  int pr = toPwm(wr * cfg_.wheel_radius, cfg_.pwm_per_mps, cfg_.max_pwm);
  const bool link_ok = link_->secondsSinceRx() < cfg_.link_timeout;
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
  int pm = blade_lockout_ ? 0 : static_cast<int>(std::clamp(blade, 0.0, 1.0) * cfg_.mow_pwm);
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
    const double dl = odo_left_.update(m.motor_pulse->left, m.motor_pulse->dir_left);
    const double dr = odo_right_.update(m.motor_pulse->right, m.motor_pulse->dir_right);
    const double dt = std::chrono::duration<double>(now - last_pulse_).count();
    if (dt > 1e-3 && dt < 1.0) {
      vel_left_ = dl / dt;
      vel_right_ = dr / dt;
    } else {
      vel_left_ = vel_right_ = 0.0;
    }
    last_pulse_ = now;
    last_.motor_pulse = m.motor_pulse;
  }
  if (m.battery) battery_ = m.battery;
  if (m.motor_current) last_.motor_current = m.motor_current;
  if (m.motor_pwm) last_.motor_pwm = m.motor_pwm;
  if (m.digital) last_.digital = m.digital;
  if (m.analog) last_.analog = m.analog;
  if (m.boundary) last_.boundary = m.boundary;
  if (m.motor_state) last_.motor_state = m.motor_state;
  if (m.power_state) last_.power_state = m.power_state;
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
    st.motor_state = motor_state = last_.motor_state.value_or("");
    st.power_state = last_.power_state.value_or("");
    st.left_pwm_cmd = last_pwm_l_;
    st.right_pwm_cmd = last_pwm_r_;
    st.mow_pwm_cmd = last_pwm_mow_;
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
      st.in_charger = b.in_charger ? *b.in_charger != 0 : b.ma > 0;
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

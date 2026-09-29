#pragma once
// ros2_control SystemInterface for the Worx mainboard (JSON over SPI).
//
// Joints (names configurable): two wheels with velocity command and
// position/velocity state, and the blade with an effort command in [0, 1].
// Wheel speeds become PWM open loop, as in the ROS1 worx_comms:
// pwm = v[m/s] * pwm_per_mps, clamped to +-max_pwm. Odometry comes from the
// MotorPulse tick counters (wheel_ticks_per_m).
//
// Besides the joints, an internal node publishes /power (BatteryState),
// /power/charger_present (Bool, used by the docking plugin), /worx/status and
// offers /worx/emergency (SetBool, latched), /worx/motors_enabled (SetBool) and
// /worx/manual_mow (SetBool: blade on by hand at the runtime parameter manual_mow_pwm,
// whose sign is the blade direction).

#include "worx_hardware/speed_controller.hpp"
#include "worx_hardware/wheel_odometer.hpp"
#include "worx_hardware/worx_link.hpp"
#include "worx_hardware/worx_protocol.hpp"

#include "open_mower_next/msg/worx_status.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

namespace open_mower_next::worx_hardware
{

class WorxSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~WorxSystem() override;

private:
  struct Config
  {
    std::string transport = "spidev";  // spidev | fake
    std::string spi_device = "/dev/spidev0.0";
    int spi_speed_hz = 1500000;
    std::string left_joint = "left_wheel_joint";
    std::string right_joint = "right_wheel_joint";
    std::string mower_joint = "mower_joint";
    double wheel_radius = 0.1;         // only for rad <-> m of the joint interfaces
    double wheel_ticks_per_m = 414.0;
    int tick_counter_bits = 32;
    double pwm_per_mps = 1230.0;       // ROS1 MAXSPEED
    int max_pwm = 1230;
    int mow_pwm = 1850;                // blade PWM at effort 1.0
    bool blade_enabled = true;         // false: blade PWM always 0 (tests near the dock)
    int manual_mow_pwm = 1850;         // blade PWM for /worx/manual_mow, sign = direction
    int manual_mow_max_pwm = 1850;     // |manual_mow_pwm| limit
    bool invert_left = false;
    bool invert_right = false;
    double link_timeout = 1.0;         // s without board messages -> link_ok false
    double blade_idle_timeout = 25.0;  // s without wheel motion commands -> blade off (ROS1 worx_comms)
    double battery_empty_voltage = 21.7;
    double battery_full_voltage = 28.5;
    std::set<std::string> digital_inverted{"Door", "Door2", "Lift", "Collision"};
    bool log_packets = true;           // throttled INFO dump of board traffic (bug hunting)
    bool bump_detection = true;
    int dock_wiggle_pwm = 150;         // one wheel back this much for 60 ms to clear BlockForward at the dock
    double bump_min_speed = 0.05;      // m/s before BlockForward counts as a bump
    double collision_hold = 1.0;       // s the bump latch holds after the last evidence
    bool lift_emergency = true;        // Lift latches an emergency (cleared via /worx/emergency)
    double resend_period = 0.2;        // s: repeat an unchanged SETSPEED
    // The firmware's InCharger never clears after leaving the dock: the charge
    // current tells instead (as ROS1 worx_comms). A full battery on the dock reads 0 mA.
    bool charger_from_current = true;
    int charger_min_ma = 1;
    // Closed-loop wheel speed (speed_controller.hpp); gains are also runtime
    // parameters of the worx_hardware node (speed_kp, speed_ki, speed_i_max).
    bool speed_control = true;
    SpeedGains speed_gains;
    double speed_filter_tau = 0.25;    // s
  };

  void onBoardMessage(const std::string & msg);
  void publishStatus();
  void sendSpeed(int left, int right, int mow, bool force);
  void registerBump(const char * source);  // with mutex_ held
  bool manualMowRefused(std::string & why) const;
  void startNode();
  void stopNode();

  Config cfg_;
  std::unique_ptr<WorxLink> link_;

  // Filled by the link thread, consumed by read()/status.
  mutable std::mutex mutex_;
  WheelOdometer odo_left_, odo_right_;
  // Wheel travel [m] = ticks / ticks_per_m * wheel_scale_ (/worx/wheel_scale, learned
  // from RTK by the wheel_scale node: grass height, debris and wear change the radius).
  double dist_left_ = 0.0, dist_right_ = 0.0;
  double wheel_scale_ = 1.0;
  double vel_left_ = 0.0, vel_right_ = 0.0;  // m/s
  SpeedFilter filt_left_, filt_right_;       // low-passed vel_left_/vel_right_ (board frame)
  std::chrono::steady_clock::time_point last_pulse_{};
  BoardMessage last_;  // merged latest values of every block
  std::optional<Battery> battery_;

  // Commands.
  WheelSpeedPI pi_left_, pi_right_;  // write() only
  std::mutex gains_mutex_;
  SpeedGains gains_;                 // gains_mutex_
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_;
  int last_pwm_l_ = 0, last_pwm_r_ = 0, last_pwm_mow_ = 0;
  bool sent_once_ = false;
  std::chrono::steady_clock::time_point last_send_{};
  // Bump latch: set by onBoardMessage, cleared in write() once the wheels were
  // commanded non-forward and collision_hold passed without new evidence.
  std::atomic<bool> collision_{false};
  std::atomic<bool> lift_{false};
  std::atomic<uint32_t> bumps_{0};
  std::chrono::steady_clock::time_point last_bump_{};  // mutex_
  std::chrono::steady_clock::time_point last_moving_{};  // mutex_: wheel speed > bump_min_speed
  std::chrono::steady_clock::time_point last_motion_cmd_ = std::chrono::steady_clock::now();
  std::atomic<bool> emergency_{false};
  std::atomic<bool> motors_enabled_{true};
  // Final docking approach (/worx/docking_mode, set by docking_helper): the dock
  // presses the bumper, the firmware then blocks forward PWM until it gets a zero
  // command. In docking mode that is no bump: one zero cycle clears the latch and
  // the approach pushes on (real robot 2026-09-28: stopped 15-20 cm short).
  std::atomic<bool> docking_mode_{false};
  std::chrono::steady_clock::time_point dock_clear_until_{};
  bool dock_wiggle_left_ = false;
  std::atomic<int> dock_wiggles_{0};
  std::chrono::steady_clock::time_point dock_next_wiggle_{};
  std::atomic<bool> active_{false};
  // Blade stays off after emergency/deactivate/motor disable/link loss until its
  // command has been <= 0 once (the effort controller holds the last command).
  std::atomic<bool> blade_lockout_{true};
  // Blade switched on by hand (/worx/manual_mow): overrides the mower joint's effort
  // with manual_mow_pwm_. Anything that locks the blade out also switches it off.
  std::atomic<bool> manual_mow_{false};
  std::atomic<int> manual_mow_pwm_{0};

  // Internal node.
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_pub_;
  rclcpp::Publisher<open_mower_next::msg::WorxStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_srv_, motors_srv_, docking_srv_, manual_mow_srv_,
    fake_charger_srv_, fake_collision_srv_, fake_lift_srv_;
  FakeBoardTransport * fake_board_ = nullptr;  // owned by link_, only with transport=fake
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fake_battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fake_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_scale_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace open_mower_next::worx_hardware

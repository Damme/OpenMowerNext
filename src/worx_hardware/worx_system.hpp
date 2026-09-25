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
// offers /worx/emergency (SetBool, latched) and /worx/motors_enabled (SetBool).

#include "worx_hardware/wheel_odometer.hpp"
#include "worx_hardware/worx_link.hpp"
#include "worx_hardware/worx_protocol.hpp"

#include "open_mower_next/msg/worx_status.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
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
    bool invert_left = false;
    bool invert_right = false;
    double link_timeout = 1.0;         // s without board messages -> link_ok false
    double blade_idle_timeout = 25.0;  // s without wheel motion commands -> blade off (ROS1 worx_comms)
    double battery_empty_voltage = 21.7;
    double battery_full_voltage = 28.5;
    std::set<std::string> digital_inverted{"Door", "Door2", "Lift", "Collision"};
    bool log_packets = true;           // throttled INFO dump of board traffic (bug hunting)
  };

  void onBoardMessage(const std::string & msg);
  void publishStatus();
  void sendSpeed(int left, int right, int mow, bool force);
  void startNode();
  void stopNode();

  Config cfg_;
  std::unique_ptr<WorxLink> link_;

  // Filled by the link thread, consumed by read()/status.
  mutable std::mutex mutex_;
  WheelOdometer odo_left_, odo_right_;
  double vel_left_ = 0.0, vel_right_ = 0.0;  // m/s
  std::chrono::steady_clock::time_point last_pulse_{};
  BoardMessage last_;  // merged latest values of every block
  std::optional<Battery> battery_;

  // Commands.
  int last_pwm_l_ = 0, last_pwm_r_ = 0, last_pwm_mow_ = 0;
  bool sent_once_ = false;
  std::chrono::steady_clock::time_point last_motion_cmd_ = std::chrono::steady_clock::now();
  std::atomic<bool> emergency_{false};
  std::atomic<bool> motors_enabled_{true};
  std::atomic<bool> active_{false};
  // Blade stays off after emergency/deactivate/motor disable/link loss until its
  // command has been <= 0 once (the effort controller holds the last command).
  std::atomic<bool> blade_lockout_{true};

  // Internal node.
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_pub_;
  rclcpp::Publisher<open_mower_next::msg::WorxStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_srv_, motors_srv_, fake_charger_srv_;
  FakeBoardTransport * fake_board_ = nullptr;  // owned by link_, only with transport=fake
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fake_battery_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace open_mower_next::worx_hardware

// ROS 2 publisher for the INA226 on the Worx robot's I2C0 (0x45, 20 mOhm shunt
// in the battery line): sensor_msgs/BatteryState on /power/ina226, 2 Hz by
// default. current_sign turns the chip's sign into BatteryState's (positive =
// charging): on MrChoppie the chip reads negative while charging. A component
// (open_mower_next::ina226::Ina226Node) with a wall timer; if the chip can't be
// set up or stops answering it retries every 5 s.
#include "ina226/ina226.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <chrono>
#include <cmath>
#include <limits>

namespace open_mower_next::ina226
{

class Ina226Node : public rclcpp::Node
{
public:
  explicit Ina226Node(const rclcpp::NodeOptions & options) : rclcpp::Node("ina226", options)
  {
    device_ = declare_parameter("i2c_device", std::string("/dev/i2c-0"));
    address_ = static_cast<uint8_t>(declare_parameter("i2c_address", 0x45));
    shunt_ohms_ = declare_parameter("shunt_ohms", 0.02);
    current_sign_ = declare_parameter("current_sign", -1.0);
    averages_ = static_cast<int>(declare_parameter("averages", 64));
    conversion_us_ = static_cast<int>(declare_parameter("conversion_us", 1100));
    const double rate = declare_parameter("rate", 2.0);

    msg_.present = true;
    msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg_.temperature = msg_.charge = msg_.capacity = msg_.design_capacity = msg_.percentage = nan;

    pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/power/ina226", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
  }

private:
  void tick()
  {
    const auto t = now();
    if (!chip_) {
      if (last_try_.nanoseconds() != 0 && (t - last_try_).seconds() < 5.0) return;
      last_try_ = t;
      std::string error;
      auto bus = openLinuxI2c(device_, address_, error);
      if (bus) {
        auto candidate = std::make_unique<Ina226>(std::move(bus), shunt_ohms_);
        if (candidate->init(Ina226::configWord(averages_, conversion_us_), error)) chip_ = std::move(candidate);
      }
      if (!chip_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 60000, "INA226 %s @0x%02X: %s", device_.c_str(), address_,
          error.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "INA226 %s @0x%02X ready - publishing /power/ina226", device_.c_str(), address_);
    }

    Reading r;
    if (!chip_->read(r)) {
      RCLCPP_WARN(get_logger(), "INA226 read failed - setting it up again");
      chip_.reset();
      last_try_ = t;
      return;
    }
    msg_.header.stamp = t;
    msg_.voltage = static_cast<float>(r.bus_voltage);
    msg_.current = static_cast<float>(current_sign_ * r.current);
    pub_->publish(msg_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 60000, "INA226 | %.3f V | %+.3f A | %.1f W", r.bus_voltage,
      current_sign_ * r.current, std::abs(r.power));
  }

  std::string device_;
  uint8_t address_;
  double shunt_ohms_;
  double current_sign_;
  int averages_;
  int conversion_us_;
  std::unique_ptr<Ina226> chip_;
  rclcpp::Time last_try_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::BatteryState msg_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace open_mower_next::ina226

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::ina226::Ina226Node)

// ROS 2 publisher for the LSM6DSV on the Worx robot: sensor_msgs/Imu on
// imu/data_raw (orientation not provided), 50 Hz by default.
#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <thread>

using open_mower_next::lsm6dsv_imu::Lsm6dsv;
using open_mower_next::lsm6dsv_imu::Sample;

namespace
{
// Datasheet noise (Table 3) at ~50 Hz bandwidth.
constexpr double GYRO_VAR = 3.5e-4 * 3.5e-4;   // (rad/s)^2
constexpr double ACCEL_VAR = 4.2e-3 * 4.2e-3;  // (m/s^2)^2
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lsm6dsv_imu");
  const auto device = node->declare_parameter("i2c_device", std::string("/dev/i2c-1"));
  const auto address = static_cast<uint8_t>(node->declare_parameter("i2c_address", 0x6B));
  const bool auto_level = node->declare_parameter("auto_level", true);
  const auto frame_id = node->declare_parameter("frame_id", std::string("imu"));
  const double rate = node->declare_parameter("rate", 50.0);
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());

  auto log = [&node](int level, const std::string & m) {
    if (level >= 2) {
      RCLCPP_ERROR(node->get_logger(), "%s", m.c_str());
    } else if (level == 1) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "%s", m.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(), "%s", m.c_str());
    }
  };

  std::string error;
  auto bus = open_mower_next::lsm6dsv_imu::openLinuxI2c(device, address, error);
  if (!bus) {
    RCLCPP_FATAL(node->get_logger(), "Cannot open I2C %s @0x%02X: %s", device.c_str(), address, error.c_str());
    rclcpp::shutdown();
    return 1;
  }
  Lsm6dsv imu(std::move(bus), log);
  if (!imu.init()) {
    rclcpp::shutdown();
    return 1;
  }
  imu.enableSflp();
  if (!imu.calibrateGyroBias() || !imu.calibrateLevel(auto_level)) {
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "LSM6DSV ready - publishing imu/data_raw at %.0f Hz", rate);

  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = frame_id;
  msg.orientation_covariance[0] = -1.0;  // orientation not provided
  for (int i : {0, 4, 8}) {
    msg.angular_velocity_covariance[i] = GYRO_VAR;
    msg.linear_acceleration_covariance[i] = ACCEL_VAR;
  }

  std::thread spinner([node]() { rclcpp::spin(node); });
  rclcpp::WallRate loop(rate);
  while (rclcpp::ok()) {
    Sample s;
    if (imu.read(s)) {
      msg.header.stamp = node->now();
      msg.angular_velocity.x = s.gyro.x;
      msg.angular_velocity.y = s.gyro.y;
      msg.angular_velocity.z = s.gyro.z;
      msg.linear_acceleration.x = s.accel.x;
      msg.linear_acceleration.y = s.accel.y;
      msg.linear_acceleration.z = s.accel.z;
      pub->publish(msg);
      const double d = 180.0 / M_PI;
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 60000,
        "IMU | gyro[dps] %+.2f %+.2f %+.2f | accel[m/s^2] %+.2f %+.2f %+.2f |a|=%.2f | T=%.1fC",
        s.gyro.x * d, s.gyro.y * d, s.gyro.z * d, s.accel.x, s.accel.y, s.accel.z,
        std::sqrt(s.accel.x * s.accel.x + s.accel.y * s.accel.y + s.accel.z * s.accel.z), s.temperature);
    }
    loop.sleep();
  }
  spinner.join();
  rclcpp::shutdown();
  return 0;
}

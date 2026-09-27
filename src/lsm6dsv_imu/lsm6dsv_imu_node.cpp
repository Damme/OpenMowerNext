// ROS 2 publisher for the LSM6DSV on the Worx robot: sensor_msgs/Imu on
// imu/data_raw (orientation not provided), 50 Hz by default. With use_fifo each
// message is the mean of all chip samples since the previous one, and the gyro
// bias tracks the chip's SFLP estimate (see lsm6dsv.hpp). A component
// (open_mower_next::lsm6dsv_imu::Lsm6dsvImuNode) with its own sensor thread;
// if the chip can't be set up it retries every 5 s.
#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

namespace open_mower_next::lsm6dsv_imu
{
namespace
{
// Datasheet noise (Table 3) at ~50 Hz bandwidth.
constexpr double GYRO_VAR = 3.5e-4 * 3.5e-4;   // (rad/s)^2
constexpr double ACCEL_VAR = 4.2e-3 * 4.2e-3;  // (m/s^2)^2
}  // namespace

class Lsm6dsvImuNode : public rclcpp::Node
{
public:
  explicit Lsm6dsvImuNode(const rclcpp::NodeOptions & options) : rclcpp::Node("lsm6dsv_imu", options)
  {
    device_ = declare_parameter("i2c_device", std::string("/dev/i2c-1"));
    address_ = static_cast<uint8_t>(declare_parameter("i2c_address", 0x6B));
    auto_level_ = declare_parameter("auto_level", true);
    frame_id_ = declare_parameter("frame_id", std::string("imu"));
    rate_ = declare_parameter("rate", 50.0);
    use_fifo_ = declare_parameter("use_fifo", true);
    fifo_opts_.sflp_bias = declare_parameter("sflp_bias", fifo_opts_.sflp_bias);
    fifo_opts_.bias_agree_dps = declare_parameter("sflp_bias_agree_dps", fifo_opts_.bias_agree_dps);
    fifo_opts_.bias_band_dps = declare_parameter("sflp_bias_band_dps", fifo_opts_.bias_band_dps);
    fifo_opts_.bias_tau = declare_parameter("sflp_bias_tau", fifo_opts_.bias_tau);
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    thread_ = std::thread([this]() { run(); });
  }

  ~Lsm6dsvImuNode() override
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
  }

private:
  bool running() const { return rclcpp::ok() && !stop_; }

  void waitRetry()
  {
    for (int i = 0; i < 50 && running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void run()
  {
    auto log = [this](int level, const std::string & m) {
      if (level >= 2) {
        RCLCPP_ERROR(get_logger(), "%s", m.c_str());
      } else if (level == 1) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", m.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", m.c_str());
      }
    };

    std::unique_ptr<Lsm6dsv> imu;
    while (running() && !imu) {
      std::string error;
      auto bus = openLinuxI2c(device_, address_, error);
      if (!bus) {
        RCLCPP_ERROR(get_logger(), "Cannot open I2C %s @0x%02X: %s", device_.c_str(), address_, error.c_str());
        waitRetry();
        continue;
      }
      auto candidate = std::make_unique<Lsm6dsv>(std::move(bus), log);
      if (!candidate->init()) {
        waitRetry();
        continue;
      }
      if (
        !candidate->calibrateGyroBias() || !candidate->calibrateLevel(auto_level_) ||
        (use_fifo_ && !candidate->enableFifo(fifo_opts_)))
      {
        waitRetry();
        continue;
      }
      imu = std::move(candidate);
    }
    if (!imu) return;
    RCLCPP_INFO(get_logger(), "LSM6DSV ready - publishing imu/data_raw at %.0f Hz", rate_);

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id_;
    msg.orientation_covariance[0] = -1.0;  // orientation not provided
    for (int i : {0, 4, 8}) {
      msg.angular_velocity_covariance[i] = GYRO_VAR;
      msg.linear_acceleration_covariance[i] = ACCEL_VAR;
    }

    rclcpp::WallRate loop(rate_);
    while (running()) {
      Sample s;
      if (imu->read(s)) {
        msg.header.stamp = now();
        msg.angular_velocity.x = s.gyro.x;
        msg.angular_velocity.y = s.gyro.y;
        msg.angular_velocity.z = s.gyro.z;
        msg.linear_acceleration.x = s.accel.x;
        msg.linear_acceleration.y = s.accel.y;
        msg.linear_acceleration.z = s.accel.z;
        pub_->publish(msg);
        const double d = 180.0 / M_PI;
        const auto ab = imu->activeBiasDps();
        const auto sb = imu->sflpBiasDps();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 60000,
          "IMU | gyro[dps] %+.3f %+.3f %+.3f | accel[m/s^2] %+.2f %+.2f %+.2f | T=%.1fC | n=%d | "
          "bias[dps] %+.4f %+.4f %+.4f (%s) sflp %+.4f %+.4f %+.4f",
          s.gyro.x * d, s.gyro.y * d, s.gyro.z * d, s.accel.x, s.accel.y, s.accel.z, s.temperature,
          s.gyro_samples, ab.x, ab.y, ab.z, imu->sflpBiasInUse() ? "SFLP" : "startup", sb.x, sb.y, sb.z);
      }
      loop.sleep();
    }
  }

  std::string device_;
  uint8_t address_;
  bool auto_level_;
  std::string frame_id_;
  double rate_;
  bool use_fifo_;
  FifoOptions fifo_opts_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace open_mower_next::lsm6dsv_imu

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::lsm6dsv_imu::Lsm6dsvImuNode)

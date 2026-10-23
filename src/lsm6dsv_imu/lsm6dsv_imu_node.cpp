// ROS 2 publisher for the LSM6DSV on the Worx robot: sensor_msgs/Imu on
// imu/data_raw (orientation not provided), 50 Hz by default. With use_fifo each
// message is the mean of all chip samples since the previous one, and the gyro
// bias can track the chip's SFLP estimate (see lsm6dsv.hpp; off by default). A
// component (open_mower_next::lsm6dsv_imu::Lsm6dsvImuNode) with its own sensor
// thread; if the chip can't be set up it retries every 5 s.
//
// Level: mount_up (body-frame gravity measured once, scripts/worx_imu_mount.py)
// is a fixed rotation; without it auto_level measures whatever the robot stands
// on at startup. Gyro bias: calibrated standing still at startup and on
// imu/calibrate (std_srvs/Trigger, answered once done, ~3 s; the message has the
// bias and the measured "up" for mount_up). After the chip stopped delivering
// and was re-initialized imu/needs_calibration (latched) is true until a
// calibration succeeded: mower_logic stops and calls imu/calibrate.
#include "lsm6dsv_imu/lsm6dsv.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

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
    auto_level_ = declare_parameter("auto_level", false);
    mount_up_ = declare_parameter<std::vector<double>>("mount_up", std::vector<double>{});
    // Standing still: the 120 Hz gyro's std is ~1 LSB (0.0175 dps) on the dock.
    calib_max_std_dps_ = declare_parameter("calibration_max_std_dps", 0.1);
    calib_max_change_dps_ = declare_parameter("calibration_max_change_dps", 0.5);
    frame_id_ = declare_parameter("frame_id", std::string("imu"));
    rate_ = declare_parameter("rate", 50.0);
    use_fifo_ = declare_parameter("use_fifo", true);
    fifo_opts_.sflp_bias = declare_parameter("sflp_bias", fifo_opts_.sflp_bias);
    fifo_opts_.bias_agree_dps = declare_parameter("sflp_bias_agree_dps", fifo_opts_.bias_agree_dps);
    fifo_opts_.bias_band_dps = declare_parameter("sflp_bias_band_dps", fifo_opts_.bias_band_dps);
    fifo_opts_.bias_tau = declare_parameter("sflp_bias_tau", fifo_opts_.bias_tau);
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    needs_pub_ = create_publisher<std_msgs::msg::Bool>("imu/needs_calibration", rclcpp::QoS(1).transient_local());
    calib_srv_ = create_service<std_srvs::srv::Trigger>(
      "imu/calibrate", [this](std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv,
                              std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<std_srvs::srv::Trigger::Request>) {
        std::lock_guard<std::mutex> l(calib_mutex_);
        calib_srv_handle_ = srv;
        calib_requests_.push_back(*header);
      });
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
      if (!candidate->calibrateGyroBias(256, calib_max_std_dps_)) {
        waitRetry();
        continue;
      }
      if (mount_up_.size() == 3) {
        candidate->setLevel({mount_up_[0], mount_up_[1], mount_up_[2]});
      } else {
        if (!mount_up_.empty()) RCLCPP_ERROR(get_logger(), "mount_up needs 3 values (x y z), got %zu", mount_up_.size());
        if (!auto_level_) {
          RCLCPP_WARN(get_logger(), "No mount_up and auto_level off: frame not levelled (yaw rate reads cos(tilt) low)");
        }
        if (!candidate->calibrateLevel(auto_level_)) {
          waitRetry();
          continue;
        }
      }
      if (use_fifo_ && !candidate->enableFifo(fifo_opts_)) {
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
      serveCalibration(*imu);
      publishNeedsCalibration(imu->needsCalibration());
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

  // Latched; published when it changes (a chip that stays away re-inits every 3 reads).
  void publishNeedsCalibration(bool needed)
  {
    if (needs_published_ && *needs_published_ == needed) return;
    needs_published_ = needed;
    std_msgs::msg::Bool m;
    m.data = needed;
    needs_pub_->publish(m);
  }

  // Sensor thread: answer pending imu/calibrate requests (one calibration for all of them).
  void serveCalibration(Lsm6dsv & imu)
  {
    std::vector<rmw_request_id_t> requests;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv;
    {
      std::lock_guard<std::mutex> l(calib_mutex_);
      requests.swap(calib_requests_);
      srv = calib_srv_handle_;
    }
    if (requests.empty()) return;
    std_srvs::srv::Trigger::Response res;
    res.success = imu.calibrateGyroBias(256, calib_max_std_dps_, calib_max_change_dps_);
    Vec3 up;
    const bool have_up = imu.measureUp(up, 64);
    char b[200];
    if (res.success) {
      const auto bias = imu.gyroBiasDps();
      std::snprintf(b, sizeof(b), "bias_dps %+.4f %+.4f %+.4f", bias.x, bias.y, bias.z);
      res.message = b;
      publishNeedsCalibration(imu.needsCalibration());  // before the answer: mower_logic then sees it cleared
    } else {
      res.message = imu.lastError();
    }
    if (have_up) {
      std::snprintf(b, sizeof(b), "; up %+.4f %+.4f %+.4f", up.x, up.y, up.z);
      res.message += b;
    }
    RCLCPP_INFO(get_logger(), "imu/calibrate: %s: %s", res.success ? "done" : "failed", res.message.c_str());
    for (auto & h : requests) srv->send_response(h, res);
  }

  std::string device_;
  uint8_t address_;
  bool auto_level_;
  std::vector<double> mount_up_;
  double calib_max_std_dps_, calib_max_change_dps_;
  std::string frame_id_;
  double rate_;
  bool use_fifo_;
  FifoOptions fifo_opts_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr needs_pub_;
  std::optional<bool> needs_published_;  // sensor thread
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_srv_;
  std::mutex calib_mutex_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> calib_srv_handle_;
  std::vector<rmw_request_id_t> calib_requests_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace open_mower_next::lsm6dsv_imu

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::lsm6dsv_imu::Lsm6dsvImuNode)

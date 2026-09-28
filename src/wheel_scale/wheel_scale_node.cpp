// wheel_scale: learns the metres per tick-metre of the Worx wheels from RTK
// (wheel_scale.hpp) and hands it to worx_hardware (/worx/wheel_scale, latched),
// which scales its odometry and the distance it drives. The value is kept in
// scale_file across restarts.
//
// In:  /joint_states (wheel travel as worx_hardware reports it, i.e. already
//      scaled - divided back by the scale in use), /imu/data_raw (yaw from the
//      gyro), /odometry/gps (RTK fixed only: docking_helper's gate).
#include "wheel_scale/wheel_scale.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fstream>
#include <optional>

namespace open_mower_next::wheel_scale
{

class WheelScaleNode : public rclcpp::Node
{
public:
  explicit WheelScaleNode(const rclcpp::NodeOptions & options) : Node("wheel_scale", options)
  {
    Params p;
    p.rate = declare_parameter("rate", p.rate);
    p.min_speed = declare_parameter("min_speed", p.min_speed);
    p.min_dist = declare_parameter("min_dist", p.min_dist);
    p.agree_tol = declare_parameter("agree_tol", p.agree_tol);
    p.straight_tol = declare_parameter("straight_tol", p.straight_tol);
    p.max_hdg = declare_parameter("max_hdg", p.max_hdg);
    p.max_asym = declare_parameter("max_asym", p.max_asym);
    p.min_scale = declare_parameter("min_scale", p.min_scale);
    p.max_scale = declare_parameter("max_scale", p.max_scale);
    p.settle_time = declare_parameter("settle_time", p.settle_time);
    p.confirm_time = declare_parameter("confirm_time", p.confirm_time);
    p.gps_gap = declare_parameter("gps_gap", p.gps_gap);
    file_ = declare_parameter("scale_file", std::string("/data/wheel_scale.txt"));
    enabled_ = declare_parameter("enabled", true);
    left_ = declare_parameter("left_joint", std::string("left_wheel_joint"));
    right_ = declare_parameter("right_joint", std::string("right_wheel_joint"));
    radius_ = declare_parameter("wheel_radius", 0.1);

    double start = 1.0;
    if (std::ifstream in(file_); in >> start) {
      RCLCPP_INFO(get_logger(), "Wheel scale %.4f from %s", start, file_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "No %s: wheel scale starts at 1.0", file_.c_str());
      start = 1.0;
    }
    cal_ = Calibrator(p, start);

    pub_ = create_publisher<std_msgs::msg::Float64>("/worx/wheel_scale", rclcpp::QoS(1).transient_local());
    publish();

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onJoints(*m); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        const rclcpp::Time t(m->header.stamp);
        if (imu_t_ && t > *imu_t_ && (t - *imu_t_).seconds() < 0.2) yaw_ += m->angular_velocity.z * (t - *imu_t_).seconds();
        imu_t_ = t;
        cal_.onHeading(yaw_);
      });
    gps_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (!enabled_) return;
        const bool changed = cal_.onGps(now().seconds(), m->pose.pose.position.x, m->pose.pose.position.y);
        if (!cal_.lastMessage().empty()) RCLCPP_INFO(get_logger(), "wheel scale: %s", cal_.lastMessage().c_str());
        if (changed) {
          publish();
          if (std::ofstream out(file_); out) out << cal_.scale() << "\n";
        }
      });
  }

private:
  void onJoints(const sensor_msgs::msg::JointState & m)
  {
    double l = NAN, r = NAN;
    for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
      if (m.name[i] == left_) l = m.position[i] * radius_;
      if (m.name[i] == right_) r = m.position[i] * radius_;
    }
    if (!std::isfinite(l) || !std::isfinite(r)) return;
    const rclcpp::Time t(m.header.stamp);
    if (last_ && t > last_->t) {
      // Divide back by the scale worx_hardware applied: the calibrator wants raw ticks.
      const double s = applied_;
      cal_.onWheels((l - last_->l) / s, (r - last_->r) / s, (t - last_->t).seconds());
    }
    last_ = Last{t, l, r};
  }

  void publish()
  {
    std_msgs::msg::Float64 msg;
    msg.data = cal_.scale();
    applied_ = msg.data;
    pub_->publish(msg);
  }

  struct Last
  {
    rclcpp::Time t;
    double l, r;
  };
  Calibrator cal_;
  std::string file_, left_, right_;
  bool enabled_ = true;
  double radius_ = 0.1, yaw_ = 0.0, applied_ = 1.0;
  std::optional<rclcpp::Time> imu_t_;
  std::optional<Last> last_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
};

}  // namespace open_mower_next::wheel_scale

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::wheel_scale::WheelScaleNode)

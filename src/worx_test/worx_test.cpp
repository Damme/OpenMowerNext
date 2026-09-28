// worx_test: short, measured test drives on the real Worx robot (Daniel at the
// robot with the STOP button). Starts in ~1 s (no python), switches the motors
// on for the test and always off again (end, error, Ctrl-C).
//
//   worx_test drive <m> [speed]                 straight, holds the gyro heading
//   worx_test turn <deg>                        in place, + = left
//   worx_test square <side_m> [right|left] [speed]
//
// Drives by wheel ticks (/joint_states), turns by the gyro (/imu/data_raw),
// commands /cmd_vel_joy (twist_mux, highest priority). Before and after each
// segment it stands 1 s and averages the GPS (/gps/fix); at the end it prints
// ticks vs GPS distance, gyro vs GPS heading change (from the legs' GPS tracks)
// and the square's closure error. Blade: never touched.
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

constexpr double kWheelRadius = 0.1;   // worx.yaml wheel.radius (joint rad <-> m)
constexpr double kLinAccel = 0.3;      // m/s^2 (= velocity_smoother)
constexpr double kAngAccel = 1.0;      // rad/s^2
constexpr double kTurnSpeed = 0.5;     // rad/s
constexpr double kHeadingGain = 2.0;   // rad/s per rad of heading error while driving
constexpr double kRate = 50.0;         // Hz
constexpr double kStale = 0.5;         // s without wheel/IMU data -> abort

double deg(double rad) { return rad * 180.0 / M_PI; }
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct GpsAvg
{
  double e = 0, n = 0;
  int count = 0;
  int worst_status = 2;  // ubx_gps NavSatStatus: 2 = RTK fixed, 0 = float or plain fix, -1 none
  bool ok() const { return count > 0; }
};

struct Segment
{
  std::string kind;  // drive | turn
  double cmd = 0;    // m or rad
  double left = 0, right = 0, gyro = 0;
  GpsAvg before, after;
};

class Tester : public rclcpp::Node
{
public:
  Tester() : Node("worx_test")
  {
    auto qos = rclcpp::SensorDataQoS();
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
        if (m->name[i] == "left_wheel_joint") left_ = m->position[i] * kWheelRadius;
        if (m->name[i] == "right_wheel_joint") right_ = m->position[i] * kWheelRadius;
      }
      js_time_ = now();
    });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
      const rclcpp::Time t(m->header.stamp);
      if (imu_last_stamp_ && t > *imu_last_stamp_) {
        const double dt = (t - *imu_last_stamp_).seconds();
        if (dt < 0.2) yaw_ += m->angular_velocity.z * dt;
      }
      imu_last_stamp_ = t;
      imu_time_ = now();
    });
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", 10, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
      if (m->status.status < 0 || !std::isfinite(m->latitude)) return;
      if (!lat0_) {
        lat0_ = m->latitude;
        lon0_ = m->longitude;
      }
      constexpr double R = 6378137.0;
      const double e = (m->longitude - *lon0_) * M_PI / 180.0 * R * std::cos(*lat0_ * M_PI / 180.0);
      const double n = (m->latitude - *lat0_) * M_PI / 180.0 * R;
      if (gps_collect_) {
        gps_acc_.e += e;
        gps_acc_.n += n;
        ++gps_acc_.count;
        gps_acc_.worst_status = std::min<int>(gps_acc_.worst_status, m->status.status);
      }
      gps_time_ = now();
    });
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_joy", 10);
    motors_ = create_client<std_srvs::srv::SetBool>("/worx/motors_enabled");
  }

  // Spin for `seconds` publishing (v, w); throws on Ctrl-C.
  void hold(double seconds, double v = 0.0, double w = 0.0)
  {
    const auto end = now() + rclcpp::Duration::from_seconds(seconds);
    rclcpp::Rate rate(kRate);
    while (now() < end) {
      tick();
      publish(v, w);
      rate.sleep();
    }
  }

  void waitForData()
  {
    const auto end = now() + rclcpp::Duration::from_seconds(10.0);
    rclcpp::Rate rate(kRate);
    while (now() < end) {
      tick();
      if (js_time_ && imu_time_) return;
      rate.sleep();
    }
    throw std::runtime_error("no /joint_states or /imu/data_raw (stack running?)");
  }

  void setMotors(bool on)
  {
    if (!motors_->wait_for_service(std::chrono::seconds(3))) throw std::runtime_error("/worx/motors_enabled not available");
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = on;
    auto fut = motors_->async_send_request(req);
    exec_->spin_until_future_complete(fut, std::chrono::seconds(3));
    if (fut.wait_for(std::chrono::seconds(0)) != std::future_status::ready || !fut.get()->success) {
      throw std::runtime_error(std::string("motors ") + (on ? "on" : "off") + " failed");
    }
    std::printf("motors %s\n", on ? "ON" : "off");
  }

  GpsAvg gpsWindow(double seconds)
  {
    gps_acc_ = {};
    gps_collect_ = true;
    hold(seconds);
    gps_collect_ = false;
    GpsAvg r = gps_acc_;
    if (r.count) {
      r.e /= r.count;
      r.n /= r.count;
    } else {
      r.worst_status = -1;
    }
    return r;
  }

  Segment drive(double dist, double vmax)
  {
    Segment s{"drive", dist};
    s.before = gpsWindow(1.0);
    const double l0 = left_, r0 = right_, y0 = yaw_;
    const double dir = dist >= 0 ? 1.0 : -1.0;
    const double len = std::abs(dist);
    double v = 0.0;
    const auto deadline = now() + rclcpp::Duration::from_seconds(len / vmax * 2.0 + 10.0);
    rclcpp::Rate rate(kRate);
    while (true) {
      tick();
      const double done = dir * 0.5 * ((left_ - l0) + (right_ - r0));
      const double rem = len - done;
      if (rem <= 0.002) break;
      if (now() > deadline) throw std::runtime_error("drive timed out");
      v = std::min({vmax, v + kLinAccel / kRate, std::sqrt(2.0 * kLinAccel * rem)});
      v = std::max(v, 0.03);
      const double w = std::clamp(kHeadingGain * wrap(y0 - yaw_), -0.3, 0.3);
      publish(dir * v, w);
      rate.sleep();
    }
    hold(0.5);  // come to rest
    s.after = gpsWindow(1.0);
    s.left = left_ - l0;
    s.right = right_ - r0;
    s.gyro = yaw_ - y0;
    report(s);
    return s;
  }

  Segment turn(double angle)
  {
    Segment s{"turn", angle};
    s.before = gpsWindow(1.0);
    const double l0 = left_, r0 = right_, y0 = yaw_;
    const double dir = angle >= 0 ? 1.0 : -1.0;
    double w = 0.0;
    const auto deadline = now() + rclcpp::Duration::from_seconds(std::abs(angle) / kTurnSpeed * 2.0 + 10.0);
    rclcpp::Rate rate(kRate);
    while (true) {
      tick();
      const double rem = std::abs(angle) - dir * (yaw_ - y0);
      if (rem <= deg2rad(0.3)) break;
      if (now() > deadline) throw std::runtime_error("turn timed out");
      w = std::min({kTurnSpeed, w + kAngAccel / kRate, std::sqrt(2.0 * kAngAccel * rem)});
      w = std::max(w, 0.15);
      publish(0.0, dir * w);
      rate.sleep();
    }
    hold(0.5);
    s.after = gpsWindow(1.0);
    s.left = left_ - l0;
    s.right = right_ - r0;
    s.gyro = yaw_ - y0;
    report(s);
    return s;
  }

  void stopNow()
  {
    for (int i = 0; i < 5; ++i) {
      publish(0.0, 0.0);
      exec_->spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void setExecutor(rclcpp::Executor * e) { exec_ = e; }

private:
  static double deg2rad(double d) { return d * M_PI / 180.0; }

  void tick()
  {
    exec_->spin_some();
    if (g_stop) throw std::runtime_error("interrupted");
    if (js_time_ && imu_time_) {
      const double js_age = (now() - *js_time_).seconds(), imu_age = (now() - *imu_time_).seconds();
      if (js_age > kStale || imu_age > kStale) {
        throw std::runtime_error("stale sensor data (joint_states " + std::to_string(js_age) + " s, imu " + std::to_string(imu_age) + " s)");
      }
    }
  }

  void publish(double v, double w)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.twist.linear.x = v;
    m.twist.angular.z = w;
    cmd_pub_->publish(m);
  }

  static void report(const Segment & s)
  {
    if (s.kind == "drive") {
      std::printf("drive %+.2f m: ticks L %+.3f R %+.3f mean %+.3f | gyro %+.1f deg", s.cmd, s.left, s.right,
        0.5 * (s.left + s.right), deg(s.gyro));
    } else {
      std::printf("turn %+.1f deg: gyro %+.1f deg | ticks L %+.3f R %+.3f", deg(s.cmd), deg(s.gyro), s.left, s.right);
    }
    if (s.before.ok() && s.after.ok()) {
      std::printf(" | GPS %.3f m (status %d/%d)", std::hypot(s.after.e - s.before.e, s.after.n - s.before.n),
        s.before.worst_status, s.after.worst_status);
    } else {
      std::printf(" | no GPS");
    }
    std::printf("\n");
    std::fflush(stdout);
  }

  rclcpp::Executor * exec_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motors_;
  double left_ = 0, right_ = 0, yaw_ = 0;
  std::optional<rclcpp::Time> js_time_, imu_time_, gps_time_, imu_last_stamp_;
  std::optional<double> lat0_, lon0_;
  bool gps_collect_ = false;
  GpsAvg gps_acc_;
};

void summary(const std::vector<Segment> & segs)
{
  std::printf("\n=== summary (GPS = 1 s averages at rest; status 2 = RTK fixed, 0 = float/3D) ===\n");
  double tick_sum = 0, gps_sum = 0;
  std::vector<std::pair<size_t, double>> track;  // segment index, GPS heading of a drive
  for (size_t i = 0; i < segs.size(); ++i) {
    const auto & s = segs[i];
    if (s.kind != "drive" || !s.before.ok() || !s.after.ok()) continue;
    const double de = s.after.e - s.before.e, dn = s.after.n - s.before.n;
    const double g = std::hypot(de, dn), t = std::abs(0.5 * (s.left + s.right));
    const double h = std::atan2(dn, de) + (s.cmd < 0 ? M_PI : 0.0);
    std::printf("leg %zu: ticks %.3f m  GPS %.3f m  (%+.1f %%)  L-R %+.3f m  GPS heading %.1f deg\n", i, t, g,
      g > 0 ? (t / g - 1.0) * 100.0 : 0.0, std::abs(s.left) - std::abs(s.right), deg(wrap(h)));
    tick_sum += t;
    gps_sum += g;
    track.emplace_back(i, h);
  }
  if (gps_sum > 0.5) {
    std::printf("all legs: ticks %.3f m / GPS %.3f m -> ticks_per_m %.1f (now 414)\n", tick_sum, gps_sum, 414.0 * tick_sum / gps_sum);
  }
  for (size_t k = 0; k + 1 < track.size(); ++k) {
    double gyro = 0;
    for (size_t j = track[k].first + 1; j < track[k + 1].first; ++j) gyro += segs[j].gyro;
    const double truth = wrap(track[k + 1].second - track[k].second);
    std::printf("turn between legs %zu and %zu: gyro %+.1f deg  GPS %+.1f deg  (gyro/GPS %.3f)\n", track[k].first,
      track[k + 1].first, deg(gyro), deg(truth), std::abs(truth) > 0.2 ? gyro / truth : 0.0);
  }
  if (segs.size() > 1 && segs.front().before.ok() && segs.back().after.ok()) {
    std::printf("closure (GPS, end - start): %.3f m\n",
      std::hypot(segs.back().after.e - segs.front().before.e, segs.back().after.n - segs.front().before.n));
  }
}

int usage()
{
  std::fprintf(stderr,
    "usage: worx_test drive <m> [speed]\n"
    "       worx_test turn <deg>              (+ = left)\n"
    "       worx_test square <side_m> [right|left] [speed]\n");
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--ros-args") break;
    args.emplace_back(argv[i]);
  }
  if (args.size() < 2) return usage();
  const std::string mode = args[0];
  double value = 0, speed = 0.15;
  bool left = false;
  try {
    value = std::stod(args[1]);
    if (mode == "drive" && args.size() > 2) speed = std::stod(args[2]);
    if (mode == "square" && args.size() > 2) left = args[2] == "left";
    if (mode == "square" && args.size() > 3) speed = std::stod(args[3]);
  } catch (const std::exception &) {
    return usage();
  }
  if (mode != "drive" && mode != "turn" && mode != "square") return usage();
  if (speed <= 0.0 || speed > 0.3 || std::abs(value) > (mode == "turn" ? 720.0 : 10.0)) {
    std::fprintf(stderr, "refusing: speed must be (0, 0.3] m/s, distance <= 10 m, turn <= 720 deg\n");
    return 2;
  }

  rclcpp::InitOptions opts;
  opts.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opts, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  auto node = std::make_shared<Tester>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  node->setExecutor(&exec);

  int rc = 0;
  bool motors_on = false;
  std::vector<Segment> segs;
  try {
    node->waitForData();
    node->setMotors(true);
    motors_on = true;
    if (mode == "drive") {
      segs.push_back(node->drive(value, speed));
    } else if (mode == "turn") {
      segs.push_back(node->turn(value * M_PI / 180.0));
    } else {
      const double turn = (left ? 1.0 : -1.0) * M_PI / 2.0;
      for (int i = 0; i < 4; ++i) {
        segs.push_back(node->drive(value, speed));
        segs.push_back(node->turn(turn));
      }
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "ABORT: %s\n", e.what());
    rc = 1;
  }
  node->stopNow();
  if (motors_on) {
    g_stop = false;
    try {
      node->setMotors(false);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "WARNING: %s - run /opt/om2/motors.sh off\n", e.what());
      rc = 1;
    }
  }
  summary(segs);
  rclcpp::shutdown();
  return rc;
}

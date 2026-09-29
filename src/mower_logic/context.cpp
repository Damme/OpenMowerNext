#include "mower_logic/context.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

namespace open_mower_next::mower_logic
{

namespace
{
bool insidePolygon(double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  bool c = false;
  const auto & p = poly.points;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    if ((p[i].y > y) != (p[j].y > y) && x < (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x) c = !c;
  }
  return c;
}

double distanceToEdges(double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  double best = 1e9;
  const auto & p = poly.points;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    const double dx = p[i].x - p[j].x, dy = p[i].y - p[j].y;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 > 0 ? std::clamp(((x - p[j].x) * dx + (y - p[j].y) * dy) / l2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(x - p[j].x - t * dx, y - p[j].y - t * dy));
  }
  return best;
}
}  // namespace

Context::Context(rclcpp::Node::SharedPtr n, Params p) : node(std::move(n)), params(std::move(p))
{
  tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf);
  coverage_client = node->create_client<open_mower_next::srv::AreaCoverage>("/area_coverage");
  emergency_client_ = node->create_client<std_srvs::srv::SetBool>("/worx/emergency");
  motors_client_ = node->create_client<std_srvs::srv::SetBool>("/worx/motors_enabled");
  blade_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/mower_controller/commands", 10);
  drive_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10);

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/power", 10, [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      battery_ = m->percentage;
    });
  charger_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/power/charger_present", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      charger_ = m->data;
    });
  worx_sub_ = node->create_subscription<open_mower_next::msg::WorxStatus>(
    "/worx/status", 10, [this](open_mower_next::msg::WorxStatus::ConstSharedPtr m) {
      bool bumped = false;
      {
        std::lock_guard<std::mutex> l(mutex_);
        bumped = bumps_seen_ && m->bumps != *bumps_seen_;
        bumps_seen_ = m->bumps;
      }
      if (bumped) onBump();
      std::lock_guard<std::mutex> l(mutex_);
      emergency_ = m->emergency || m->board_emergency == 1;
      worx_emergency_ = m->emergency;
      board_emergency_ = m->board_emergency == 1;
      lift_ = m->lift;
      collision_ = m->collision;
      for (size_t i = 0; i < m->digital_names.size() && i < m->digital_active.size(); ++i) {
        if (m->digital_names[i] == "Rain" && m->digital_active[i]) last_rain_ = Clock::now();
      }
    });
  if (params.require_gps) {
    gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      params.gps_fix_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        const double acc = std::sqrt(std::max(m->position_covariance[0], m->position_covariance[4]));
        // ubx_gps: STATUS_GBAS_FIX = RTK fixed, STATUS_FIX = float or plain 3D (its
        // hAcc can't tell them apart). Localization only fuses RTK fixed, so float
        // must count as "no GPS" here too (dead reckoning for gps_timeout, then pause).
        const int min_status = params.gps_require_rtk_fixed ? sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
                                                            : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        const bool good = m->status.status >= min_status &&
                          m->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
                          acc <= params.gps_max_accuracy;
        std::lock_guard<std::mutex> l(mutex_);
        // A bad sample alone changes nothing: gpsOk() only drops the fix once no good
        // one came for gps_timeout (Daniel: ~4 s on IMU + ticks through float).
        if (good) {
          if (!gps_good_since_) gps_good_since_ = Clock::now();
          gps_last_good_ = Clock::now();
        }
      });
  }
  // Latched (costmap_layers::BumpLayer): each message replaces the last. Also
  // republished every second: marks under the robot are left out, and the
  // robot moves.
  obstacle_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/bump_obstacles",
                                                                        rclcpp::QoS(1).reliable().transient_local());
  // What was felt, for the web UI (JSON, latched, sent when it changes).
  markers_pub_ = node->create_publisher<std_msgs::msg::String>("~/obstacles", rclcpp::QoS(1).transient_local());
  obstacle_timer_ = node->create_wall_timer(std::chrono::seconds(1), [this]() {
    publishObstacles();
    publishMarkers();
  });
  erase_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() { eraseUnderRobot(); });
  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map_grid", rclcpp::QoS(1).transient_local().reliable(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      grid_ = *m;
    });
  map_sub_ = node->create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(1).transient_local().reliable(),
    [this](open_mower_next::msg::Map::ConstSharedPtr m) {
      std::lock_guard<std::mutex> l(mutex_);
      map_ = *m;
    });
  loadEdgeCorrections();
  loadObstacles();
}

void Context::loadObstacles()
{
  if (params.obstacles_file.empty()) return;
  std::ifstream f(params.obstacles_file);
  std::string line;
  const auto shape = markShape();
  std::lock_guard<std::mutex> l(mutex_);
  size_t n = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    int id = 0, state = 1;
    Contact c;
    if (!(s >> id >> c.x >> c.y >> c.yaw >> c.side)) continue;
    s >> state;  // missing (first files): confirmed
    c.id = next_contact_id_++;
    c.marks = contactMarks(c, shape);
    c.total = c.marks.size();
    auto it = std::find_if(obstacles_.begin(), obstacles_.end(), [&](const FeltObstacle & o) { return o.id == id; });
    if (it == obstacles_.end()) {
      obstacles_.push_back(FeltObstacle{id, {}});
      it = std::prev(obstacles_.end());
    }
    it->state = state == 0 ? FeltObstacle::State::PENDING
              : state == 2 ? FeltObstacle::State::GONE : FeltObstacle::State::CONFIRMED;
    if (it->state == FeltObstacle::State::GONE) c.marks.clear();
    it->contacts.push_back(c);
    next_obstacle_id_ = std::max(next_obstacle_id_, id + 1);
    ++n;
  }
  RCLCPP_INFO(node->get_logger(), "%zu felt obstacles (%zu contacts) from %s", obstacles_.size(), n,
              params.obstacles_file.c_str());
}

void Context::saveObstacles() const
{
  if (params.obstacles_file.empty()) return;
  // Temp file + rename: a crash never leaves half a file. Marks erased where the
  // robot drove are not kept: after a restart the bands are whole again.
  const auto tmp = params.obstacles_file + ".tmp";
  {
    std::ofstream f(tmp);
    f << "# obstacle x y yaw side state - felt obstacles: robot pose at each bump (map frame), state 0 pending,"
         " 1 confirmed, 2 gone; see mower_logic\n";
    for (const auto & o : obstacles_) {
      const int st = o.state == FeltObstacle::State::PENDING ? 0 : o.state == FeltObstacle::State::CONFIRMED ? 1 : 2;
      for (const auto & c : o.contacts) {
        f << o.id << ' ' << c.x << ' ' << c.y << ' ' << c.yaw << ' ' << c.side << ' ' << st << '\n';
      }
    }
  }
  std::rename(tmp.c_str(), params.obstacles_file.c_str());
}

void Context::loadEdgeCorrections()
{
  if (params.edge_corrections_file.empty()) return;
  std::ifstream f(params.edge_corrections_file);
  std::string line;
  std::lock_guard<std::mutex> l(mutex_);
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    EdgeCorrection c;
    if (s >> c.x >> c.y >> c.offset) edge_corrections_.push_back(c);
  }
  RCLCPP_INFO(node->get_logger(), "%zu edge corrections from %s", edge_corrections_.size(),
              params.edge_corrections_file.c_str());
}

void Context::saveEdgeCorrections() const
{
  if (params.edge_corrections_file.empty()) return;
  // Write a temp file and rename it, so a crash never leaves half a file.
  const auto tmp = params.edge_corrections_file + ".tmp";
  {
    std::ofstream f(tmp);
    f << "# x y inward_offset (m, map frame) - perimeter bump corrections, see mower_logic\n";
    for (const auto & c : edge_corrections_) f << c.x << ' ' << c.y << ' ' << c.offset << '\n';
  }
  std::rename(tmp.c_str(), params.edge_corrections_file.c_str());
}

double Context::distanceToLines(double x, double y) const
{
  double best = 1e9;
  for (const auto & a : map_.areas) {
    if (a.area.polygon.points.size() >= 3) best = std::min(best, distanceToEdges(x, y, a.area.polygon));
  }
  return best;
}

bool Context::insideAreas(double x, double y) const
{
  bool in = false;
  for (const auto & a : map_.areas) {
    if (a.area.polygon.points.size() < 3 || !insidePolygon(x, y, a.area.polygon)) continue;
    if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION) return false;
    in = true;
  }
  return in;
}

void Context::shiftOutline(std::vector<open_mower_next::msg::CoveragePath> & passes, double cx, double cy,
                           double delta) const
{
  const double full = params.edge_correction_radius, fade = params.edge_correction_ramp;
  for (auto & pass : passes) {
    if (!pass.is_outline) continue;
    auto & poses = pass.path.poses;
    bool changed = false;
    for (auto & ps : poses) {
      auto & p = ps.pose.position;
      const double d = std::hypot(p.x - cx, p.y - cy);
      if (d >= full + fade) continue;
      // Only the loop riding the line; inner loops are further in already.
      const double edge = distanceToLines(p.x, p.y);
      if (edge > params.edge_correction_loop_distance) continue;
      const double w = d <= full ? 1.0 : 1.0 - (d - full) / fade;
      const auto & q = ps.pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      // Inward = the side of the path moving away from the nearest line.
      const double nx = -std::sin(yaw), ny = std::cos(yaw);
      const double left = distanceToLines(p.x + 0.05 * nx, p.y + 0.05 * ny);
      const double right = distanceToLines(p.x - 0.05 * nx, p.y - 0.05 * ny);
      const double side = left >= right ? 1.0 : -1.0;
      const double sx = p.x + side * delta * w * nx, sy = p.y + side * delta * w * ny;
      if (!insideAreas(sx, sy)) continue;
      p.x = sx;
      p.y = sy;
      changed = true;
    }
    if (!changed) continue;
    // Headings follow the shifted path.
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto & a = poses[i == 0 ? 0 : i - 1].pose.position;
      const auto & b = poses[i + 1 < poses.size() ? i + 1 : i].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) < 1e-6) continue;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      poses[i].pose.orientation.x = poses[i].pose.orientation.y = 0.0;
      poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }
  }
}

std::optional<double> Context::addEdgeCorrection(double x, double y, double yaw)
{
  const double cx = x + params.edge_correction_ahead * std::cos(yaw);
  const double cy = y + params.edge_correction_ahead * std::sin(yaw);
  double delta = 0.0, total = 0.0;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (distanceToLines(x, y) > params.edge_bump_distance) return std::nullopt;
    auto it = std::find_if(edge_corrections_.begin(), edge_corrections_.end(), [&](const EdgeCorrection & c) {
      return std::hypot(c.x - cx, c.y - cy) < params.edge_correction_radius;
    });
    if (it == edge_corrections_.end()) {
      edge_corrections_.push_back({cx, cy, 0.0});
      it = std::prev(edge_corrections_.end());
    }
    if (it->offset + 1e-6 >= params.edge_correction_max) return std::nullopt;
    total = std::min(params.edge_correction_max, it->offset + params.edge_correction_step);
    delta = total - it->offset;
    it->offset = total;
    saveEdgeCorrections();
    ++markers_version_;
    // The existing spot keeps its centre so the shift already applied stays consistent.
    const double sx = it->x, sy = it->y;
    mission.editPlan([&](auto & passes) { shiftOutline(passes, sx, sy, delta); });
  }
  return total;
}

void Context::applyEdgeCorrections(std::vector<open_mower_next::msg::CoveragePath> & passes) const
{
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & c : edge_corrections_) shiftOutline(passes, c.x, c.y, c.offset);
}

void Context::dropBumpObstacle(const Bump & b)
{
  // The whole obstacle: it was the edge, not something on the lawn.
  {
    std::lock_guard<std::mutex> l(mutex_);
    obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(),
                                    [&](const FeltObstacle & o) { return o.id == b.obstacle; }),
                     obstacles_.end());
    ++markers_version_;
    saveObstacles();
  }
  publishObstacles();
}

double Context::batteryFraction() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return battery_;
}

bool Context::charging() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return charger_;
}

bool Context::atDock() const
{
  if (charging()) return true;
  const auto pose = robotPose();
  std::lock_guard<std::mutex> l(mutex_);
  if (!pose) return false;
  for (const auto & d : map_.docking_stations) {
    const auto & p = d.pose.pose.position;
    if (std::hypot(p.x - pose->pose.position.x, p.y - pose->pose.position.y) < params.undock_distance) return true;
  }
  return false;
}

bool Context::emergency() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return emergency_;
}

bool Context::clearEmergency(std::string & message)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (lift_) {
      message = "refused: the robot is lifted (Lift input active)";
      return false;
    }
    if (collision_) {
      message = "refused: bumper still pressed";
      return false;
    }
    if (board_emergency_) {
      // Tilt with the blade on: the firmware cut the motor MOSFET (MOTORREQ_EMGSTOP) and
      // only accepts ping/ENABLE/DISABLE/SETSPEED over SPI, so it can't be reset from here.
      message = "refused: firmware emergency (tilt) - needs MOTORREQ_RESETEMG support in the firmware or a board reset";
      return false;
    }
    if (!worx_emergency_) {
      message = "no emergency latched";
      return true;
    }
  }
  if (!emergency_client_->service_is_ready()) {
    message = "/worx/emergency not available";
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = false;
  emergency_client_->async_send_request(req);
  message = "emergency cleared";
  return true;
}

bool Context::raining()
{
  if (!params.dock_on_rain) return false;
  std::lock_guard<std::mutex> l(mutex_);
  return last_rain_ && std::chrono::duration<double>(Clock::now() - *last_rain_).count() < params.rain_clear_delay;
}

bool Context::gpsOk()
{
  if (!params.require_gps) return true;
  std::lock_guard<std::mutex> l(mutex_);
  const auto now = Clock::now();
  if (!gps_last_good_ || std::chrono::duration<double>(now - *gps_last_good_).count() > params.gps_timeout) {
    gps_good_since_.reset();
    return false;
  }
  return gps_good_since_ && std::chrono::duration<double>(now - *gps_good_since_).count() >= params.gps_settle;
}

bool Context::needsCharging()
{
  std::lock_guard<std::mutex> l(mutex_);
  if (std::isnan(battery_)) return needs_charging_;
  // Low only when it stays low: the voltage sags for seconds under load (blade
  // spin-up, uphill), and one sample below battery_low must not end the mowing.
  const auto now = Clock::now();
  if (battery_ >= params.battery_low) {
    low_since_.reset();
  } else if (!low_since_) {
    low_since_ = now;
  }
  if (low_since_ && std::chrono::duration<double>(now - *low_since_).count() >= params.battery_low_time) {
    needs_charging_ = true;
  }
  if (battery_ >= params.battery_resume) needs_charging_ = false;
  return needs_charging_;
}

std::vector<std::string> Context::operationAreas() const
{
  std::vector<std::string> disabled;
  if (!params.disabled_areas_file.empty()) {
    std::ifstream in(params.disabled_areas_file);
    for (std::string line; std::getline(in, line);) {
      line = line.substr(0, line.find('#'));
      const auto b = line.find_first_not_of(" \t\r");
      if (b == std::string::npos) continue;
      disabled.push_back(line.substr(b, line.find_last_not_of(" \t\r") - b + 1));
    }
  }
  auto listed = [](const std::vector<std::string> & v, const std::string & s) {
    return !s.empty() && std::find(v.begin(), v.end(), s) != v.end();
  };
  std::lock_guard<std::mutex> l(mutex_);
  std::vector<std::string> ids;
  for (const auto & a : map_.areas) {
    if (a.type != open_mower_next::msg::Area::TYPE_OPERATION) continue;
    if (!params.areas.empty() && !listed(params.areas, a.id)) continue;
    if (listed(disabled, a.id)) {
      RCLCPP_INFO(node->get_logger(), "Area %s disabled in %s", a.id.c_str(),
                  params.disabled_areas_file.c_str());
      continue;
    }
    ids.push_back(a.id);
  }
  return ids;
}

std::optional<geometry_msgs::msg::PoseStamped> Context::robotPose() const
{
  try {
    const auto t = tf->lookupTransform("map", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped p;
    p.header = t.header;
    p.pose.position.x = t.transform.translation.x;
    p.pose.position.y = t.transform.translation.y;
    p.pose.orientation = t.transform.rotation;
    return p;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void Context::drive(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "base_link";
  t.twist.linear.x = linear;
  t.twist.angular.z = angular;
  drive_pub_->publish(t);
}

void Context::setBlade(bool on)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (on != blade_on_) {
      RCLCPP_INFO(node->get_logger(), "Blade %s", on ? "ON" : "OFF");
    }
    blade_on_ = on;
  }
  std_msgs::msg::Float64MultiArray m;
  m.data = {on ? 1.0 : 0.0};
  blade_pub_->publish(m);
}

bool Context::setMotors(bool on)
{
  if (!motors_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "Motors %s: /worx/motors_enabled not available",
                         on ? "on" : "off");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Motors %s", on ? "on" : "off");
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  motors_client_->async_send_request(req);
  return true;
}

std::string Context::lastBranch() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return branch_;
}

void Context::setBranch(const std::string & b)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (b != branch_) RCLCPP_INFO(node->get_logger(), "State: %s", b.c_str());
  branch_ = b;
}

MarkShape Context::markShape() const
{
  MarkShape m;
  m.body.front = params.footprint_front;
  m.body.rear = params.footprint_rear;
  m.body.half_width = params.footprint_half_width;
  m.body.chamfer = params.footprint_front_chamfer;
  m.gap = params.bump_mark_gap;
  m.depth = params.bump_mark_depth;
  return m;
}

namespace
{
// Middle of a contact's band: where the bumper was at the bump.
Point contactCentre(const Contact & c, const MarkShape & s)
{
  const double d = s.body.front + s.gap + s.depth / 2.0;
  return {c.x + d * std::cos(c.yaw), c.y + d * std::sin(c.yaw)};
}

const char * stateName(FeltObstacle::State s)
{
  switch (s) {
    case FeltObstacle::State::PENDING: return "pending";
    case FeltObstacle::State::CONFIRMED: return "confirmed";
    default: return "gone";
  }
}
}  // namespace

void Context::onBump()
{
  const auto pose = robotPose();
  if (!pose) {
    RCLCPP_WARN(node->get_logger(), "Bump, but no robot pose - obstacle not marked");
    std::lock_guard<std::mutex> l(mutex_);
    last_bump_ = Bump{Clock::now(), NAN, NAN};
    return;
  }
  const auto & q = pose->pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  Contact c;
  c.x = pose->pose.position.x;
  c.y = pose->pose.position.y;
  c.yaw = yaw;
  c.side = bump_side.load();
  c.marks = contactMarks(c, markShape());
  c.total = c.marks.size();
  Bump b;
  b.time = Clock::now();
  b.x = c.x;
  b.y = c.y;
  b.yaw = yaw;
  int contacts = 0;
  {
    std::lock_guard<std::mutex> l(mutex_);
    c.id = next_contact_id_++;
    b.contact = c.id;
    // The obstacle being felt around, else the nearest one (not gone) whose
    // marks come within bump_merge_distance of the new ones, else a new one.
    FeltObstacle * into = nullptr;
    const int feeling = feel_obstacle.load();
    for (auto & o : obstacles_) {
      if (o.id == feeling && o.state != FeltObstacle::State::GONE) into = &o;
    }
    if (!into) {
      double best = params.bump_merge_distance;
      for (auto & o : obstacles_) {
        if (o.state == FeltObstacle::State::GONE) continue;
        for (const auto & [px, py] : c.marks) {
          const double d = o.distance(px, py);
          if (d <= best) {
            best = d;
            into = &o;
          }
        }
      }
    }
    if (!into) {
      obstacles_.push_back(FeltObstacle{next_obstacle_id_++, {}, FeltObstacle::State::PENDING});
      into = &obstacles_.back();
      b.first = true;
    } else if (into->state == FeltObstacle::State::PENDING) {
      into->state = FeltObstacle::State::CONFIRMED;  // bumped again at the same spot
    }
    // Bumping the same spot again from the same pose refreshes that contact.
    auto same = std::find_if(into->contacts.begin(), into->contacts.end(),
                             [&](const Contact & o) { return sameContact(o, c); });
    if (same != into->contacts.end()) into->contacts.erase(same);
    into->contacts.push_back(c);
    if (static_cast<int>(into->contacts.size()) > params.bump_max_contacts) into->contacts.erase(into->contacts.begin());
    b.obstacle = into->id;
    b.confirmed = into->state == FeltObstacle::State::CONFIRMED;
    contacts = static_cast<int>(into->contacts.size());
    last_bump_ = b;
    ++markers_version_;
    saveObstacles();
  }
  const char * where = c.side > 0 ? "front left" : c.side < 0 ? "front right" : "front";
  if (b.first) {
    RCLCPP_WARN(node->get_logger(), "Bump at (%.2f, %.2f), %s: obstacle %d, not believed yet", c.x, c.y, where,
                b.obstacle);
  } else {
    RCLCPP_WARN(node->get_logger(), "Bump at (%.2f, %.2f), %s: obstacle %d confirmed, %d touches", c.x, c.y, where,
                b.obstacle, contacts);
  }
  publishObstacles();
}

void Context::eraseUnderRobot()
{
  const auto pose = robotPose();
  if (!pose) return;
  const auto & q = pose->pose.orientation;
  const double x = pose->pose.position.x, y = pose->pose.position.y;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  // Only while driving forward (both wheels): only then would the bumper have
  // reported what the body now covers. Turning in place or reversing can sweep
  // past a pin unnoticed (sim: a sidestepped pin was erased by a turn).
  const auto last = std::exchange(erase_last_, std::array<double, 3>{x, y, yaw});
  const double ds = (x - last[0]) * std::cos(yaw) + (y - last[1]) * std::sin(yaw);
  const double dyaw = std::abs(std::remainder(yaw - last[2], 2.0 * M_PI));
  if (!std::isfinite(last[0]) || ds < 0.003 || ds < dyaw * params.footprint_half_width) return;
  const auto shape = markShape();
  auto inside = [&](double px, double py) { return insideBody(shape.body, x, y, yaw, px, py); };
  std::vector<std::string> events;
  {
    std::lock_guard<std::mutex> l(mutex_);
    bool changed = false, state_changed = false;
    for (auto & o : obstacles_) {
      if (o.state == FeltObstacle::State::GONE) continue;
      if (o.erase(inside)) changed = true;
      char text[160];
      if (o.state == FeltObstacle::State::PENDING) {
        // Driven clearly through where the bumper was (15 cm past it: the pose
        // at the bump lags a little): nothing there.
        const auto & c0 = o.contacts.front();
        const double ahead = shape.body.front + 0.15;
        const Point ctr{c0.x + ahead * std::cos(c0.yaw), c0.y + ahead * std::sin(c0.yaw)};
        if (inside(ctr.first, ctr.second)) {
          std::snprintf(text, sizeof(text), "Bump %d at (%.2f, %.2f) was a false detection: drove through there", o.id,
                        ctr.first, ctr.second);
          events.emplace_back(text);
          o.state = FeltObstacle::State::GONE;
        }
      } else if (o.markCount() == 0) {
        std::snprintf(text, sizeof(text), "Obstacle %d isn't there any more: drove through all of it", o.id);
        events.emplace_back(text);
        o.state = FeltObstacle::State::GONE;
      }
      if (o.state == FeltObstacle::State::GONE) {
        for (auto & c : o.contacts) c.marks.clear();
        state_changed = changed = true;
      }
    }
    if (changed) ++markers_version_;
    if (state_changed) saveObstacles();
  }
  for (const auto & e : events) RCLCPP_WARN(node->get_logger(), "%s", e.c_str());
}

std::optional<int> Context::obstacleNear(double x, double y, double radius) const
{
  std::lock_guard<std::mutex> l(mutex_);
  std::optional<int> id;
  double best = radius;
  const int probing = probing_obstacle.load();
  for (const auto & o : obstacles_) {
    if (o.state != FeltObstacle::State::CONFIRMED || o.id == probing) continue;
    const double d = o.distance(x, y);
    if (d <= best) {
      best = d;
      id = o.id;
    }
  }
  return id;
}

double Context::obstacleDistance(int id, double x, double y) const
{
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & o : obstacles_) {
    if (o.id == id) return o.distance(x, y);
  }
  return std::numeric_limits<double>::infinity();
}

size_t Context::obstacleContacts(int id) const
{
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & o : obstacles_) {
    if (o.id == id) return o.contacts.size();
  }
  return 0;
}

FeltObstacle::State Context::obstacleState(int id) const
{
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & o : obstacles_) {
    if (o.id == id) return o.state;
  }
  return FeltObstacle::State::GONE;
}

std::optional<size_t> Context::sidestep(int id, double x, double y, double offset, double min_clearance)
{
  const auto pass = mission.currentPass(0.0);
  if (!pass || pass->path.poses.size() < 3) return std::nullopt;
  const auto & poses = pass->path.poses;
  std::vector<PathPose> pp;
  pp.reserve(poses.size());
  for (const auto & ps : poses) {
    const auto & o = ps.pose.orientation;
    pp.push_back({ps.pose.position.x, ps.pose.position.y,
                  std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))});
  }
  // The pose next to the spot, within the next 4 m of the pass.
  size_t c = 0;
  double best = 1e9, along = 0.0;
  for (size_t i = 0; i < pp.size() && along <= 4.0; ++i) {
    const double d = std::hypot(pp[i].x - x, pp[i].y - y);
    if (d < best) {
      best = d;
      c = i;
    }
    if (i + 1 < pp.size()) along += std::hypot(pp[i + 1].x - pp[i].x, pp[i + 1].y - pp[i].y);
  }
  if (best > 1.0) return std::nullopt;
  const auto [first, last] = jogPath(pp, 0, c, offset, params.feel_sidestep_length, params.feel_sidestep_ramp);
  if (first > last) return std::nullopt;
  for (size_t i = first; i <= last; ++i) {
    if (!footprintFits(pp[i].x, pp[i].y, pp[i].yaw)) return std::nullopt;
  }
  // Clear of the obstacle along the new line, also just outside the shifted part.
  if (min_clearance > 0.0) {
    for (size_t i = first > 20 ? first - 20 : 0; i < pp.size() && i <= last + 20; ++i) {
      if (obstacleDistance(id, pp[i].x, pp[i].y) < min_clearance) return std::nullopt;
    }
  }
  const size_t start = pass->start_index;
  const size_t a = first > 0 ? first - 1 : 0, b = std::min(pp.size() - 1, last + 1);
  bool written = false;
  mission.editCurrentPass([&](nav_msgs::msg::Path & path, size_t) {
    if (start + b >= path.poses.size()) return;
    for (size_t i = a; i <= b; ++i) {
      auto & ps = path.poses[start + i].pose;
      ps.position.x = pp[i].x;
      ps.position.y = pp[i].y;
      ps.orientation.x = ps.orientation.y = 0.0;
      ps.orientation.z = std::sin(pp[i].yaw / 2.0);
      ps.orientation.w = std::cos(pp[i].yaw / 2.0);
    }
    written = true;
  });
  if (!written) return std::nullopt;
  return start + last;
}

std::optional<Context::Bump> Context::lastBump() const
{
  std::lock_guard<std::mutex> l(mutex_);
  return last_bump_;
}

bool Context::bumpedSince(Clock::time_point t) const
{
  std::lock_guard<std::mutex> l(mutex_);
  return last_bump_ && last_bump_->time > t;
}

std::optional<Context::Bump> Context::takeBump()
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!last_bump_ || last_bump_->time <= handled_bump_) return std::nullopt;
  handled_bump_ = last_bump_->time;
  return last_bump_;
}

void Context::clearBumpObstacles()
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!obstacles_.empty()) {
      RCLCPP_INFO(node->get_logger(), "Forgetting %zu felt obstacles", obstacles_.size());
      ++markers_version_;
    }
    obstacles_.clear();
    probing_obstacle = -1;
    if (last_bump_) handled_bump_ = last_bump_->time;
    saveObstacles();
  }
  publishObstacles();
  publishMarkers();
}

bool Context::forgetObstacle(double x, double y, double radius, bool all, std::string & message)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (all) {
      message = "forgot " + std::to_string(obstacles_.size()) + " felt obstacle(s)";
      obstacles_.clear();
    } else {
      // By its marks, or where the bumper was (a gone one has no marks left).
      const auto shape = markShape();
      auto best = obstacles_.end();
      double best_d = radius;
      for (auto it = obstacles_.begin(); it != obstacles_.end(); ++it) {
        double d = it->distance(x, y);
        for (const auto & c : it->contacts) {
          const auto ctr = contactCentre(c, shape);
          d = std::min(d, std::hypot(ctr.first - x, ctr.second - y));
        }
        if (d <= best_d) {
          best_d = d;
          best = it;
        }
      }
      if (best == obstacles_.end()) {
        message = "no felt obstacle there";
        return false;
      }
      message = "forgot obstacle " + std::to_string(best->id) + " (" + stateName(best->state) + ", " +
                std::to_string(best->contacts.size()) + " touches)";
      obstacles_.erase(best);
    }
    ++markers_version_;
    saveObstacles();
  }
  publishObstacles();
  publishMarkers();
  return true;
}

bool Context::forgetEdgeCorrection(double x, double y, double radius, bool all, std::string & message)
{
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (all) {
      message = "forgot " + std::to_string(edge_corrections_.size()) + " edge correction(s)";
      edge_corrections_.clear();
    } else {
      auto best = edge_corrections_.end();
      double best_d = radius;
      for (auto it = edge_corrections_.begin(); it != edge_corrections_.end(); ++it) {
        const double d = std::hypot(it->x - x, it->y - y);
        if (d <= best_d) {
          best_d = d;
          best = it;
        }
      }
      if (best == edge_corrections_.end()) {
        message = "no edge correction there";
        return false;
      }
      char text[96];
      std::snprintf(text, sizeof(text), "forgot the edge correction at (%.2f, %.2f), %.0f cm", best->x, best->y,
                    best->offset * 100.0);
      message = text;
      edge_corrections_.erase(best);
    }
    // The plan being mowed keeps its shift; the next plan of the area won't have it.
    message += " (from the next plan of the area on)";
    saveEdgeCorrections();
    ++markers_version_;
  }
  publishMarkers();
  return true;
}

bool Context::refreshObstacles()
{
  return publishObstacles();
}

bool Context::publishObstacles()
{
  std::vector<std::pair<float, float>> pts;
  bool left_out = false;
  const auto robot = robotPose();
  double rc = 1.0, rs = 0.0;
  if (robot) {
    const auto & q = robot->pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    rc = std::cos(yaw);
    rs = std::sin(yaw);
  }
  // Inside the robot's current footprint grown by bump_keep_free?
  auto near_robot = [&](double px, double py) {
    if (!robot) return false;
    const double dx = px - robot->pose.position.x, dy = py - robot->pose.position.y;
    const double u = dx * rc + dy * rs, v = -dx * rs + dy * rc;
    const double m = params.bump_keep_free;
    return u > -params.footprint_rear - m && u < params.footprint_front + m &&
           std::abs(v) < params.footprint_half_width + m;
  };
  {
    std::lock_guard<std::mutex> l(mutex_);
    // Confirmed obstacles only (a first bump is tried again). One point per 5 cm
    // cell (the costmap resolution); marks outside every area are useless
    // (already blocked) and dropped.
    std::set<std::pair<long, long>> cells;
    for (const auto & o : obstacles_) {
      if (o.state != FeltObstacle::State::CONFIRMED) continue;
      for (const auto & c : o.contacts) {
        for (const auto & [px, py] : c.marks) {
          const std::pair<long, long> cell{std::lround(std::floor(px / 0.05)), std::lround(std::floor(py / 0.05))};
          if (cells.count(cell)) continue;
          if (near_robot(px, py)) {
            left_out = true;
            continue;
          }
          if (!insideAreas(px, py)) continue;
          cells.insert(cell);
          pts.emplace_back(static_cast<float>(px), static_cast<float>(py));
        }
      }
    }
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & [px, py] : pts) {
    *x = px;
    *y = py;
    *z = 0.1f;
    ++x, ++y, ++z;
  }
  obstacle_pub_->publish(cloud);
  return left_out;
}

void Context::publishMarkers()
{
  std::ostringstream s;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (markers_version_ == markers_published_) return;
    markers_published_ = markers_version_;
    const auto shape = markShape();
    auto num = [](double v) { return std::round(v * 100.0) / 100.0; };
    s << "{\"depth\":" << shape.depth << ",\"obstacles\":[";
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      const auto & o = obstacles_[i];
      // lines: the bumper band at each touch; cells: what is left of the marks
      // (5 cm cells), i.e. where the obstacle can still be.
      s << (i ? "," : "") << "{\"id\":" << o.id << ",\"state\":\"" << stateName(o.state) << "\",\"lines\":[";
      for (size_t k = 0; k < o.contacts.size(); ++k) {
        s << (k ? "," : "") << "[";
        const auto line = contactLine(o.contacts[k], shape);
        for (size_t j = 0; j < line.size(); ++j) {
          s << (j ? "," : "") << "[" << num(line[j].first) << "," << num(line[j].second) << "]";
        }
        s << "]";
      }
      s << "],\"cells\":[";
      std::set<std::pair<long, long>> cells;
      for (const auto & c : o.contacts) {
        for (const auto & [px, py] : c.marks) cells.insert({std::lround(std::floor(px / 0.05)), std::lround(std::floor(py / 0.05))});
      }
      bool first = true;
      for (const auto & [cx, cy] : cells) {
        s << (first ? "" : ",") << "[" << num((cx + 0.5) * 0.05) << "," << num((cy + 0.5) * 0.05) << "]";
        first = false;
      }
      s << "]}";
    }
    s << "],\"edges\":[";
    for (size_t i = 0; i < edge_corrections_.size(); ++i) {
      const auto & c = edge_corrections_[i];
      s << (i ? "," : "") << "{\"x\":" << num(c.x) << ",\"y\":" << num(c.y) << ",\"offset\":" << num(c.offset)
        << ",\"radius\":" << params.edge_correction_radius << "}";
    }
    s << "]}";
  }
  std_msgs::msg::String m;
  m.data = s.str();
  markers_pub_->publish(m);
}

bool Context::footprintFits(double x, double y, double yaw) const
{
  std::lock_guard<std::mutex> l(mutex_);
  // Centre: inside an operation/navigation area, not in an exclusion.
  bool in_area = false;
  for (const auto & a : map_.areas) {
    if (a.area.polygon.points.size() < 3 || !insidePolygon(x, y, a.area.polygon)) continue;
    if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION) return false;
    in_area = true;
  }
  if (!in_area) return false;
  const auto & info = grid_.info;
  if (grid_.data.empty() || info.resolution <= 0.0) return true;  // no grid yet: centre check only
  auto cell_ok = [&](double px, double py) {
    const int cx = static_cast<int>(std::floor((px - info.origin.position.x) / info.resolution));
    const int cy = static_cast<int>(std::floor((py - info.origin.position.y) / info.resolution));
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height)) return false;
    const int8_t v = grid_.data[static_cast<size_t>(cy) * info.width + static_cast<size_t>(cx)];
    return v >= 0 && v < 100;  // free, rim or blurred edge; not unknown, not lethal
  };
  // The footprint's outline (front corners cut at 45 deg), every 5 cm.
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double f = params.footprint_front, r = -params.footprint_rear, w = params.footprint_half_width;
  const double ch = std::clamp(params.footprint_front_chamfer, 0.0, std::min(w, f - r));
  const double outline[][2] = {{f, w - ch}, {f - ch, w}, {r, w}, {r, -w}, {f - ch, -w}, {f, -w + ch}};
  constexpr size_t n_pts = sizeof(outline) / sizeof(outline[0]);
  for (size_t i = 0; i < n_pts; ++i) {
    const double u0 = outline[i][0], v0 = outline[i][1];
    const double u1 = outline[(i + 1) % n_pts][0], v1 = outline[(i + 1) % n_pts][1];
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(u1 - u0, v1 - v0) / 0.05)));
    for (int k = 0; k <= n; ++k) {
      const double u = u0 + (u1 - u0) * k / n, v = v0 + (v1 - v0) * k / n;
      if (!cell_ok(x + u * c - v * s, y + u * s + v * c)) return false;
    }
  }
  return true;
}

std::optional<double> Context::reverseForTurn(double x, double y, double yaw, double target_yaw, double max_reverse) const
{
  const double delta = std::remainder(target_yaw - yaw, 2.0 * M_PI);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / (5.0 * M_PI / 180.0))));
  for (double d = 0.0; d <= max_reverse + 1e-9; d += 0.1) {
    const double px = x - d * std::cos(yaw), py = y - d * std::sin(yaw);
    bool ok = true;
    // The way back (same heading) and the turn in place.
    for (double b = 0.0; ok && b < d; b += 0.05) ok = footprintFits(x - b * std::cos(yaw), y - b * std::sin(yaw), yaw);
    for (int k = 0; ok && k <= steps; ++k) ok = footprintFits(px, py, yaw + delta * k / steps);
    if (ok) return d;
  }
  return std::nullopt;
}

std::optional<Context::Escape> Context::escapeFootprint(double x, double y, double yaw, double max_reverse) const
{
  for (double d = 0.05; d <= max_reverse + 1e-9; d += 0.05) {
    if (footprintFits(x - d * std::cos(yaw), y - d * std::sin(yaw), yaw)) return Escape{d, 0.0};
  }
  for (int deg = 10; deg <= 180; deg += 10) {
    for (const int sign : {1, -1}) {
      const double turn = sign * deg * M_PI / 180.0;
      if (footprintFits(x, y, yaw + turn)) return Escape{0.0, turn};
    }
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::PoseStamped> Context::transitVia(const geometry_msgs::msg::PoseStamped & goal)
{
  if (params.transit_jitter <= 0.0) return std::nullopt;
  const auto start = robotPose();
  if (!start) return std::nullopt;
  const double sx = start->pose.position.x, sy = start->pose.position.y;
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double len = std::hypot(gx - sx, gy - sy);
  if (len < params.transit_jitter_min_distance) return std::nullopt;
  const double nx = -(gy - sy) / len, ny = (gx - sx) / len;  // left normal
  std::lock_guard<std::mutex> l(mutex_);
  // Point inside a non-exclusion area and >= margin from every area edge.
  auto open_lawn = [&](double x, double y) {
    bool in_area = false;
    for (const auto & a : map_.areas) {
      if (a.area.polygon.points.size() < 3) continue;
      const bool inside = insidePolygon(x, y, a.area.polygon);
      if (a.type == open_mower_next::msg::Area::TYPE_EXCLUSION && inside) return false;
      if (a.type != open_mower_next::msg::Area::TYPE_EXCLUSION && inside) in_area = true;
      if (distanceToEdges(x, y, a.area.polygon) < params.transit_via_margin) return false;
    }
    return in_area;
  };
  // Only vary transits across open lawn; in corridors a via point makes an S-bend.
  const int steps = static_cast<int>(len / 0.5);
  for (int i = 1; i < steps; ++i) {
    const double u = static_cast<double>(i) / steps;
    if (!open_lawn(sx + u * (gx - sx), sy + u * (gy - sy))) return std::nullopt;
  }
  std::uniform_real_distribution<double> along(0.35, 0.65), side(-params.transit_jitter, params.transit_jitter);
  for (int attempt = 0; attempt < 8; ++attempt) {
    const double u = along(rng_), o = side(rng_);
    const double x = sx + u * (gx - sx) + o * nx, y = sy + u * (gy - sy) + o * ny;
    if (!open_lawn(x, y)) continue;
    geometry_msgs::msg::PoseStamped via = goal;
    via.pose.position.x = x;
    via.pose.position.y = y;
    const double yaw = std::atan2(gy - sy, gx - sx);  // heading along the transit
    via.pose.orientation.z = std::sin(yaw / 2);
    via.pose.orientation.w = std::cos(yaw / 2);
    return via;
  }
  return std::nullopt;
}

}  // namespace open_mower_next::mower_logic

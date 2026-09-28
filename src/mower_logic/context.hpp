#pragma once
// Shared state of the mower_logic executor: robot inputs (battery, charger,
// GPS, emergency, rain), the operator command, mission progress and the ROS
// handles the behaviour tree nodes use.

#include "mower_logic/mission.hpp"

#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/msg/worx_status.hpp"
#include "open_mower_next/srv/area_coverage.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace open_mower_next::mower_logic
{

enum class Command { IDLE, MOW, HOME };

inline const char * toString(Command c)
{
  switch (c) {
    case Command::MOW: return "MOW";
    case Command::HOME: return "HOME";
    default: return "IDLE";
  }
}

struct Params
{
  double battery_low = 0.20;         // go charging below this fraction
  double battery_resume = 0.95;      // resume mowing at this fraction
  bool require_gps = true;
  std::string gps_fix_topic = "/gps/fix";
  double gps_max_accuracy = 0.10;    // m (sqrt of horizontal covariance)
  double gps_timeout = 4.0;          // s without a good fix -> pause (Daniel: ~4 s on IMU + ticks)
  bool gps_require_rtk_fixed = true; // only RTK fixed is a good fix (float = dead reckoning)
  double gps_settle = 10.0;          // s of good fixes before continuing (OM_GPS_WAIT_TIME_SEC)
  bool dock_on_rain = true;
  double rain_clear_delay = 1800.0;  // s without rain before mowing again
  int max_pass_attempts = 3;
  int max_dock_attempts = 3;
  double blade_spinup = 2.0;         // s between blade on and driving
  double resume_backtrack = 0.5;     // m re-mowed before a resume point
  double resume_direct_distance = 0.3;  // m: robot this close to the pass -> no transit (0 = always transit)
  // Bumps (WorxStatus.bumps): back up, mark an obstacle for Nav2, continue past it.
  // A bump marks the contact area: three discs along the front (left corner,
  // centre, right corner) - the sensor doesn't say where it was hit, and when
  // turning it is often a corner. Points outside the areas are dropped.
  double bump_front_offset = 0.62;   // m from base_link to the disc centres (front edge 0.47)
  double bump_obstacle_radius = 0.18;
  // Never mark cells under the robot's current footprint (+ margin): with the
  // footprint inside an obstacle nothing can move ("Start occupied", back-up
  // refused) - a slow Digital report while turning put marks under the robot.
  double footprint_front = 0.47, footprint_rear = 0.11, footprint_half_width = 0.195;
  double footprint_front_chamfer = 0.10;  // m cut off each front corner at 45 deg (the real corners are round)
  double bump_keep_free = 0.1;       // m around the footprint
  double corner_max_reverse = 1.0;   // m: back up at most this far to turn at a tight corner
  double bump_clearance = 0.9;       // m: pass continues at the first pose this far from the disc centre
  double bump_backup = 0.3;          // m reversed after a bump
  double bump_backup_speed = 0.1;
  int max_bumps_per_pass = 4;
  // Known bump obstacles are avoided on later passes/loops before touching them:
  // the pass is checked this far ahead of the robot.
  double bump_merge_distance = 0.4;  // m: a bump this close to a known obstacle replaces it
  double bump_lookahead = 1.0;       // m along the pass
  double bump_avoid_radius = 0.5;    // m from a disc centre (radius + half body + margin)
  // Perimeter bumps (overgrown plants, GPS a few cm off): on an outline pass
  // with the centre this close to an edge, the outline is shifted inward
  // around the spot instead of going around an obstacle, and the correction is
  // remembered (edge_corrections_file) for later loops and missions.
  double edge_bump_distance = 0.35;       // m: centre to the nearest recorded line
  double edge_correction_step = 0.10;     // m inward per bump at the same spot
  double edge_correction_max = 0.20;      // m; bumped again beyond it: obstacle handling
  double edge_correction_radius = 0.5;    // m around the spot shifted fully
  double edge_correction_ramp = 0.5;      // m over which the shift fades out
  double edge_correction_ahead = 0.35;    // m: spot = robot centre + this along its heading
  double edge_correction_loop_distance = 0.25;  // m: only outline poses this close to a line are shifted
  std::string edge_corrections_file;      // empty: kept in memory only
  int max_skipped_passes_in_row = 3; // then stop the mission and go home (navigation keeps failing)
  std::string dock_type = "openmower";
  double undock_distance = 1.5;      // m from the dock pose: below this the robot counts as docked
  std::string transit_bt;            // navigate_through_poses tree for transits to a pass (empty = bt_navigator default)
  // Random via point on longer transits so repeated trips don't wear one track into the lawn.
  double transit_jitter = 0.35;      // m: max lateral offset of the via point (0 = off)
  double transit_jitter_min_distance = 4.0;  // m: shorter transits go direct
  // Only in open lawn: the via point AND the straight line robot->goal keep this
  // clearance (a via point in a corridor made an S-bend through it).
  double transit_via_margin = 1.2;   // m
  std::string controller_id = "FollowPath";
  std::string goal_checker_id = "general_goal_checker";
  std::string progress_checker_id = "";  // empty = controller_server default
  std::vector<std::string> areas;    // operation area ids to mow; empty = all, in map order
  // Areas to leave out (one area id per line, e.g. mow_4; '#' comments), re-read at
  // every mission start so an area can be switched off without a restart.
  std::string disabled_areas_file;   // empty: none
};

class Context
{
public:
  using Clock = std::chrono::steady_clock;

  Context(rclcpp::Node::SharedPtr node, Params params);

  rclcpp::Node::SharedPtr node;
  Params params;
  Mission mission;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::atomic<Command> command{Command::IDLE};
  std::atomic<int> dock_failures{0};
  std::atomic<int> undock_failures{0};
  std::atomic<int> skipped_passes_in_row{0};
  std::atomic<bool> blade_in_use{false};  // set by FollowPass while it runs
  std::atomic<bool> count_failure{true};  // false: last pass "failure" was an early end, resume without counting

  // ---- inputs ----
  double batteryFraction() const;
  bool charging() const;   // charger present
  // On the charger or still within undock_distance of a docking station (an
  // undock that aborted halfway leaves the robot off the charger but in the dock
  // entrance, outside the areas: nothing can be planned from there).
  bool atDock() const;
  bool emergency() const;
  // Clears the latched emergency if nothing still demands it; the message says why not.
  bool clearEmergency(std::string & message);
  bool raining();          // latched for rain_clear_delay after the last rain
  bool gpsOk();            // good fixes for gps_settle, none missing for gps_timeout
  bool needsCharging();    // latched: set below battery_low, cleared at battery_resume
  std::vector<std::string> operationAreas() const;
  std::optional<geometry_msgs::msg::PoseStamped> robotPose() const;
  // Can the mower stand at this pose? Centre inside the areas (not in an
  // exclusion), the whole footprint on known, non-lethal cells of /map_grid
  // (the areas plus the rim, map_server grid.edge_band). A recorded line can
  // turn more sharply than the 0.58 m body: the front swings out when turning.
  bool footprintFits(double x, double y, double yaw) const;
  // Distance to reverse straight back from (x, y, yaw) so the mower can turn in
  // place to target_yaw with its whole footprint inside line + rim (checked
  // every 5 deg). nullopt: not possible within max_reverse.
  std::optional<double> reverseForTurn(double x, double y, double yaw, double target_yaw, double max_reverse) const;
  // The robot's footprint left line + rim (FTC overshot a cut corner): the
  // shortest straight back-up (<= max_reverse), else the smallest turn in place,
  // after which it fits again. Nav2 plans nothing from there ("Start occupied").
  struct Escape
  {
    double reverse = 0.0;  // m
    double turn = 0.0;     // rad
  };
  std::optional<Escape> escapeFootprint(double x, double y, double yaw, double max_reverse) const;
  // Perimeter bump at robot pose (x, y, yaw) on an outline pass: records or
  // grows the edge correction there and shifts the current plan by the
  // increase. nullopt: not at an edge, or the correction is at its maximum.
  std::optional<double> addEdgeCorrection(double x, double y, double yaw);
  // Shift the outline passes of a new plan by all remembered corrections.
  void applyEdgeCorrections(std::vector<open_mower_next::msg::CoveragePath> & passes) const;
  // Random via point between the robot and goal (nullopt: go direct).
  std::optional<geometry_msgs::msg::PoseStamped> transitVia(const geometry_msgs::msg::PoseStamped & goal);

  // Bumps. Each one adds an obstacle point (published for the costmaps).
  struct Bump
  {
    Clock::time_point time;
    double x = 0, y = 0;  // obstacle (disc centre) in map
    double yaw = 0;       // robot heading at the bump
  };
  std::optional<Bump> lastBump() const;
  bool bumpedSince(Clock::time_point t) const;
  // The bump a recovery has to handle (newer than the last one handled).
  std::optional<Bump> takeBump();
  void clearBumpObstacles();
  // Forget the obstacle marked for this bump (handled as an edge correction).
  void dropBumpObstacle(const Bump & b);
  std::atomic<bool> bump_on_pass{false};  // the last interrupted action was FollowPass
  std::atomic<bool> pass_is_outline{false};  // the pass GetPass handed out last
  // Behaviour tree thread only: the last reversal (bump recovery or a blind
  // FreeFootprint back-up). Daniel 2026-09-28: after a bump it backed up three
  // times where one was enough - a blind back-up isn't repeated near the last one.
  struct Reversal
  {
    Clock::time_point time;
    double x, y;
  };
  std::optional<Reversal> last_reversal;
  bool reversedNear(double x, double y) const
  {
    return last_reversal && Clock::now() - last_reversal->time < std::chrono::seconds(60) &&
           std::hypot(x - last_reversal->x, y - last_reversal->y) < 1.0;
  }
  // Known bump obstacle within radius of (x, y), if any.
  std::optional<Bump> knownObstacleNear(double x, double y, double radius) const;
  // Behaviour tree thread only: where SkipPastBump continues the pass.
  std::optional<Bump> skip_target;
  // Set by GetPass when the pass was cut before a corner the body can't drive:
  // absolute pose index to continue at after this segment (SIZE_MAX: the rest
  // of the pass is undrivable).
  std::optional<size_t> segment_resume;
  // Heading to turn to for the corner after a cut segment (CornerTurn).
  std::optional<double> corner_yaw;
  // After a corner: continue from here (turned, up to corner_max_reverse away),
  // or force a transit (no back-up worked). Consumed by GetPass.
  bool continue_from_here = false, force_transit = false;
  bool skip_counts_as_bump = true;
  bool avoiding_known_obstacle = false;

  void setBlade(bool on);
  // Direct drive command (twist_mux navigation input), for the last-resort reverse.
  void drive(double linear, double angular);
  rclcpp::Client<open_mower_next::srv::AreaCoverage>::SharedPtr coverage_client;

  std::string lastBranch() const;
  void setBranch(const std::string & b);

private:
  void onBump();
  void publishObstacles();
  mutable std::mutex mutex_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double battery_ = NAN;
  bool charger_ = false;
  bool emergency_ = false;
  bool worx_emergency_ = false, board_emergency_ = false, lift_ = false, collision_ = false;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
  std::optional<Clock::time_point> last_rain_;
  std::optional<Clock::time_point> gps_good_since_, gps_last_good_;
  bool needs_charging_ = false;
  open_mower_next::msg::Map map_;
  nav_msgs::msg::OccupancyGrid grid_;
  std::string branch_;
  std::optional<uint32_t> bumps_seen_;
  std::mt19937 rng_{std::random_device{}()};
  std::optional<Bump> last_bump_;
  Clock::time_point handled_bump_{};
  std::vector<Bump> obstacles_;
  struct EdgeCorrection
  {
    double x = 0, y = 0, offset = 0;  // spot in map, inward shift (m)
  };
  std::vector<EdgeCorrection> edge_corrections_;
  void loadEdgeCorrections();
  void saveEdgeCorrections() const;
  // Shift outline poses near (cx, cy) inward by delta (tapered). mutex_ held.
  void shiftOutline(std::vector<open_mower_next::msg::CoveragePath> & passes, double cx, double cy,
                    double delta) const;
  double distanceToLines(double x, double y) const;  // nearest recorded line; mutex_ held
  bool insideAreas(double x, double y) const;        // mutex_ held
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
  bool blade_on_ = false;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
  rclcpp::Subscription<open_mower_next::msg::WorxStatus>::SharedPtr worx_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr blade_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;
};

}  // namespace open_mower_next::mower_logic

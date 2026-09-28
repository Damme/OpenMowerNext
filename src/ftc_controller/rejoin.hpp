#pragma once
// Rejoining the path from beside it. After a turn in place on a slope the Worx
// stood up to ~0.5 m beside the pass (real robot 2026-09-28). There the PID's
// heading and lateral terms fight: ROS1 drove parallel to the path (fixed there
// by lateral_priority_distance, upstream OpenMower #310), and with the heading
// term scaled down the lateral term (kp_lat 12) asks for ~90 deg, above
// max_drive_angle, so the robot turned in place instead of driving back.
//
// Instead, off the path the robot steers to the path heading plus an approach
// angle atan(offset / lookahead), capped at max_angle: it points at the path
// whenever it is off it (never parallel) and stays below max_drive_angle (keeps
// driving). Between blend_start and blend_full of offset the command blends from
// the normal FTC law into this one. No ROS here (unit-tested).
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace open_mower_next::ftc_controller
{

struct RejoinConfig
{
  double blend_start = 0.10;    // m of offset: normal law below
  double blend_full = 0.25;     // m: rejoin law only above
  double lookahead = 0.5;       // m: approach angle = atan(offset / lookahead)
  double max_angle_deg = 30.0;  // approach angle cap (< max_drive_angle)
};

// Offset of the robot from the carrot's path line, positive when the path lies
// to the robot's left (independent of the robot's heading).
inline double crossTrack(const Eigen::Affine3d & carrot_in_base)
{
  return -carrot_in_base.inverse().translation().y();
}

// Heading relative to the path the robot should have (positive = turned left).
inline double approachAngle(double cross_track, const RejoinConfig & c)
{
  const double cap = c.max_angle_deg * M_PI / 180.0;
  return std::clamp(std::atan2(cross_track, std::max(c.lookahead, 1e-3)), -cap, cap);
}

// 0: normal FTC law, 1: rejoin law.
inline double rejoinWeight(double cross_track, const RejoinConfig & c)
{
  if (c.blend_full <= c.blend_start) return std::abs(cross_track) >= c.blend_full ? 1.0 : 0.0;
  return std::clamp((std::abs(cross_track) - c.blend_start) / (c.blend_full - c.blend_start), 0.0, 1.0);
}

}  // namespace open_mower_next::ftc_controller

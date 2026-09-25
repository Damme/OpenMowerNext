#pragma once

#include "coverage_planner/coverage_planner.hpp"

#include "open_mower_next/msg/coverage_geometry.hpp"
#include "open_mower_next/msg/coverage_path.hpp"
#include "open_mower_next/msg/polygon_with_holes.hpp"

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>

namespace open_mower_next::coverage_server::utils
{
namespace cp = ::coverage_planner;

inline bool isValid(const geometry_msgs::msg::Polygon & polygon)
{
  return polygon.points.size() >= 3;
}

inline std::vector<cp::Pt> toPoints(const geometry_msgs::msg::Polygon & polygon)
{
  std::vector<cp::Pt> pts;
  pts.reserve(polygon.points.size());
  for (const auto & p : polygon.points) {
    pts.emplace_back(p.x, p.y);
  }
  return pts;
}

// True if the two polygons share any area or boundary.
inline bool intersects(const geometry_msgs::msg::Polygon & a, const geometry_msgs::msg::Polygon & b)
{
  const auto pa = geom::makePolygon(toPoints(a));
  const auto pb = geom::makePolygon(toPoints(b));
  if (bg::is_empty(pa) || bg::is_empty(pb)) {
    return false;
  }
  try {
    return bg::intersects(pa, pb);
  } catch (...) {
    return false;
  }
}

// Planned points -> nav path; pose yaw from coverage_planner::headings().
inline nav_msgs::msg::Path toPathMsg(
  const std::vector<cp::Pt> & pts, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  const auto yaw = cp::headings(pts);
  path.poses.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = pts[i].x;
    pose.pose.position.y = pts[i].y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw[i]);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  return path;
}

inline open_mower_next::msg::CoveragePath toMsg(
  const cp::PlannedPath & planned, const std_msgs::msg::Header & header)
{
  open_mower_next::msg::CoveragePath msg;
  switch (planned.kind) {
    case cp::PlannedPath::PERIMETER:
      msg.kind = open_mower_next::msg::CoveragePath::KIND_PERIMETER;
      break;
    case cp::PlannedPath::OBSTACLE_RING:
      msg.kind = open_mower_next::msg::CoveragePath::KIND_OBSTACLE;
      break;
    default:
      msg.kind = open_mower_next::msg::CoveragePath::KIND_FILL;
      break;
  }
  msg.is_outline = planned.is_outline;
  msg.path = toPathMsg(planned.pts, header);
  return msg;
}

inline nav_msgs::msg::Path concatenate(
  const std::vector<open_mower_next::msg::CoveragePath> & paths,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path out;
  out.header = header;
  for (const auto & p : paths) {
    out.poses.insert(out.poses.end(), p.path.poses.begin(), p.path.poses.end());
  }
  return out;
}

// Ring -> Polygon, without the closing duplicate point.
template <typename Ring>
geometry_msgs::msg::Polygon toPolygonMsg(const Ring & ring)
{
  geometry_msgs::msg::Polygon msg;
  size_t n = ring.size();
  if (n > 1 && bg::equals(ring.front(), ring.back())) {
    --n;
  }
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(bg::get<0>(ring[i]));
    p.y = static_cast<float>(bg::get<1>(ring[i]));
    msg.points.push_back(p);
  }
  return msg;
}

// Mowable area (outline minus exclusions) -> one cell per component.
inline open_mower_next::msg::CoverageGeometry toMsg(
  const cp::BMultiPolygon & mowable, const std::string & frame_id)
{
  open_mower_next::msg::CoverageGeometry msg;
  msg.header.frame_id = frame_id;
  for (const auto & poly : mowable) {
    if (bg::is_empty(poly)) {
      continue;
    }
    open_mower_next::msg::PolygonWithHoles cell;
    cell.header.frame_id = frame_id;
    cell.exterior = toPolygonMsg(poly.outer());
    for (const auto & inner : poly.inners()) {
      cell.holes.push_back(toPolygonMsg(inner));
    }
    msg.cells.push_back(cell);
  }
  return msg;
}

}  // namespace open_mower_next::coverage_server::utils

#pragma once
//
// coverage_planner.hpp - ROS-agnostic OpenMower coverage planner.
//
// Author: Daniel Wiegert. Standalone Boost.Geometry planner, written to replace
// the slic3r-based coverage planner OpenMower originally used. The same code also
// runs as a ROS1 node on the Worx mower; this is that node's core with the ROS
// parts (params, request/response types, logging, markers) moved to the caller.
//
// Per zone: [1] build field  [2+3] boundary loops + obstacle rings from shared
// per-level contours  [4] fill (serpentine cells or concentric rings)
// [5] nearest-neighbour order  [6] resample + blade->base_link shift.
//

#include <functional>
#include <string>
#include <vector>

#include "coverage_planner/geom.hpp"

namespace coverage_planner
{

using geom::BMultiPolygon;
using geom::Pt;

// Node-level settings. Names and defaults match the ROS1 node's private params.
struct Params
{
  int lane_skip = 2;                  // grass-recovery lane grouping (k)
  bool optimize_sweep_angle = true;   // true: replace the request angle with the region's principal axis
  double blade_offset_x = 0.0;        // cutting deck offset from base_link [m], body frame
  double blade_offset_y = 0.0;
  double tool_width = 0.22;           // cut width [m]; <= 0 = follow Request::distance
  double overlap = 0.35;              // swath overlap, fraction of cut width [0..0.9]
  double clearance = 0.04;            // boundary inset [m]; < 0 = follow Request::outer_offset
  double obstacle_clearance = -1.0;   // obstacle inset [m]; < 0 = follow clearance
  int outline_count = 3;              // perimeter passes; < 0 = follow Request::outline_count
  std::string fill_mode = "request";  // request | serpentine | concentric
  double path_spacing = 0.1;          // emitted point spacing + loop resample [m]
  double body_width = 0.39;
  double body_length = 0.58;
  double min_turn_radius = 0.0;
  double loop_transition = 6.0;       // ring step-over length ~ this * ring gap
  std::string edge_side = "right";    // boundary/obstacle edge is kept on this side of the mower
  double axle_from_rear = 0.11;
};

// One zone. Mirrors the ROS1 PlanPath request.
struct Request
{
  std::vector<Pt> outline;                 // zone boundary (open or closed ring)
  std::vector<std::vector<Pt>> holes;      // obstacles / exclusions; ones outside the zone are ignored
  double angle_rad = 0.0;                  // fill direction (ignored if optimize_sweep_angle)
  double distance = 0.0;                   // cut width, used when Params::tool_width <= 0
  double outer_offset = 0.0;               // boundary inset, used when Params::clearance < 0
  int outline_count = 0;                   // perimeter passes, used when Params::outline_count < 0
  bool concentric = false;                 // fill type, used when Params::fill_mode == "request"
};

struct PlannedPath
{
  enum Kind { PERIMETER, OBSTACLE_RING, FILL } kind = FILL;
  bool is_outline = false;  // perimeter + obstacle loops (mower_logic treats them differently)
  std::vector<Pt> pts;      // base_link positions, spaced ~path_spacing, >= 2 points
};

struct Result
{
  std::vector<PlannedPath> paths;  // in driving order: perimeter, then fill + obstacles by proximity
  BMultiPolygon mowable;           // outline minus obstacles, for visualisation
};

enum class LogLevel { INFO, WARN, ERROR };
using LogFn = std::function<void(LogLevel, const std::string &)>;

// Never throws; on failure logs an error and returns no paths.
Result plan(const Request & req, const Params & params, const LogFn & log = {});

// Heading per point: towards the next point, the last point keeps the previous
// heading. Same as the pose yaw the ROS1 node emits.
std::vector<double> headings(const std::vector<Pt> & pts);

}  // namespace coverage_planner

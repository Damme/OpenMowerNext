#pragma once
// Can the mower stand at / move to a pose? The body against the areas and
// /map_grid (map_server: the areas plus the rim, exclusions lethal, unknown
// outside). Pure geometry, unit tested; Context holds the map and grid and
// calls this under its mutex.
//
// Daniel 2026-10-09: no blind moves. The mower reversed "the way it came"
// along its current heading after a Nav2 spin had turned it, out of mow_2 at a
// sharp corner, and then three more times into the bushes. Every move
// mower_logic drives itself is checked here pose by pose first.

#include "mower_logic/felt_obstacles.hpp"  // Body

#include "open_mower_next/msg/map.hpp"

#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <climits>
#include <optional>

namespace open_mower_next::mower_logic
{

bool insidePolygon(double x, double y, const geometry_msgs::msg::Polygon & poly);

// A turn in place, then a straight drive along the new heading.
struct Move
{
  double turn = 0.0;   // rad
  double drive = 0.0;  // m, negative = backwards
};

class FootprintCheck
{
public:
  // misfit() of a pose whose centre is outside the areas or in an exclusion.
  static constexpr int kCentreOutside = 1000;

  FootprintCheck(const open_mower_next::msg::Map & map, const nav_msgs::msg::OccupancyGrid & grid, const Body & body)
  : map_(map), grid_(grid), body_(body) {}

  // 0: the mower fits (centre inside an operation/navigation area and not in an
  // exclusion, the outline - every 5 cm - on known, non-lethal cells). Else how
  // badly not: outline points off line + rim, plus kCentreOutside. Counting
  // stops once above `limit`.
  int misfit(double x, double y, double yaw, int limit = INT_MAX) const;
  bool fits(double x, double y, double yaw) const { return misfit(x, y, yaw, 0) == 0; }

  // How far the mower can drive straight from (x, y, yaw), forwards (dir > 0)
  // or backwards, up to max (5 cm steps) with no pose on the way worse than
  // `allowed` (misfit).
  double straightRoom(double x, double y, double yaw, int dir, double max, int allowed) const;

  // The footprint left line + rim: the smallest move after which it fits, with
  // no pose on the way (every 5 cm / 5 deg) worse than now - so never further
  // out of the area or into an exclusion. Straight back or forward (shortest,
  // back on a tie), else a turn in place (smallest), else a turn and then a
  // straight drive (smallest turn). nullopt: nothing within max_drive.
  std::optional<Move> escape(double x, double y, double yaw, double max_drive) const;

private:
  const open_mower_next::msg::Map & map_;
  const nav_msgs::msg::OccupancyGrid & grid_;
  Body body_;
};

}  // namespace open_mower_next::mower_logic

// FootprintCheck: no move mower_logic drives itself may leave line + rim or
// get worse than where the robot stands (robot 2026-10-09, mow_2).

#include "mower_logic/footprint.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

using open_mower_next::msg::Area;
using open_mower_next::msg::Map;
using open_mower_next::mower_logic::Body;
using open_mower_next::mower_logic::FootprintCheck;
using open_mower_next::mower_logic::insidePolygon;
using open_mower_next::mower_logic::Move;

namespace
{
using Pts = std::vector<std::pair<double, double>>;

// mow_2 of the robot's map (map frame, m) - its sharp west corner at (14.62, -24.95).
const Pts kMow2 = {
  {22.891, -7.161}, {23.185, -7.873}, {23.284, -8.520}, {23.255, -9.903}, {23.138, -10.913}, {22.956, -11.901},
  {23.472, -12.980}, {23.551, -13.427}, {23.528, -13.768}, {23.396, -14.189}, {23.293, -14.370}, {22.948, -14.732},
  {22.745, -14.864}, {22.318, -14.994}, {21.610, -15.329}, {20.709, -15.833}, {20.377, -15.943}, {20.167, -15.945},
  {19.949, -15.871}, {19.504, -15.465}, {19.217, -15.338}, {19.098, -15.347}, {18.881, -15.454}, {18.424, -15.842},
  {18.039, -16.095}, {17.617, -16.238}, {15.927, -16.106}, {15.035, -15.969}, {14.778, -16.142}, {14.670, -16.372},
  {14.422, -16.871}, {13.439, -17.984}, {12.639, -17.933}, {11.064, -17.301}, {10.348, -18.004}, {10.116, -19.238},
  {10.294, -19.560}, {10.688, -20.620}, {10.788, -20.816}, {11.301, -21.246}, {12.497, -21.792}, {12.814, -21.839},
  {13.266, -21.832}, {14.160, -21.748}, {14.954, -21.848}, {15.270, -21.985}, {15.509, -22.228}, {15.612, -22.416},
  {15.816, -23.050}, {15.742, -23.468}, {15.451, -23.932}, {15.277, -24.071}, {14.896, -24.522}, {14.616, -24.951},
  {14.643, -25.171}, {14.896, -25.695}, {15.463, -26.001}, {15.672, -26.044}, {17.016, -25.958}, {19.063, -25.737},
  {19.562, -25.495}, {19.806, -25.247}, {20.043, -24.723}, {21.269, -22.837}, {21.624, -22.128}, {22.744, -20.332},
  {23.109, -19.606}, {23.586, -18.909}, {23.811, -18.406}, {24.153, -17.942}, {24.536, -17.245}, {24.921, -16.681},
  {25.247, -15.976}, {25.805, -15.196}, {26.630, -13.814}, {27.147, -12.815}, {28.012, -11.428}, {28.600, -10.267},
  {28.553, -10.358}, {28.308, -10.133}, {27.935, -9.905}, {27.778, -9.768}, {27.597, -9.494}, {27.584, -8.283},
  {27.668, -7.980}, {27.914, -7.790}, {28.864, -7.592}, {29.062, -7.492}, {29.715, -7.013}, {29.933, -6.756},
  {29.942, -6.647}, {29.842, -6.463}, {29.605, -6.369}, {29.374, -6.342}, {27.847, -6.511}, {27.434, -6.370},
  {27.169, -6.147}, {26.871, -5.773}, {26.439, -4.890}, {26.111, -4.436}, {24.988, -3.260}, {24.794, -3.135},
  {24.578, -3.107}, {24.259, -3.259}, {24.197, -3.488}, {24.184, -4.045}, {24.089, -4.499}, {23.857, -5.061},
  {23.196, -6.311},
};

Area area(const Pts & pts, uint8_t type)
{
  Area a;
  a.type = type;
  for (const auto & [x, y] : pts) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    a.area.polygon.points.push_back(p);
  }
  return a;
}

// Like map_server: 5 cm cells, free inside the operation areas, lethal in
// exclusions, a 0.25 m rim (60) outside next to free cells, unknown beyond.
nav_msgs::msg::OccupancyGrid grid(const Map & map, double x0, double y0, double x1, double y1)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.resolution = 0.05;
  g.info.origin.position.x = x0;
  g.info.origin.position.y = y0;
  g.info.width = static_cast<uint32_t>((x1 - x0) / 0.05);
  g.info.height = static_cast<uint32_t>((y1 - y0) / 0.05);
  const int w = g.info.width, h = g.info.height;
  g.data.assign(static_cast<size_t>(w) * h, -1);
  for (int cy = 0; cy < h; ++cy) {
    for (int cx = 0; cx < w; ++cx) {
      const double x = x0 + (cx + 0.5) * 0.05, y = y0 + (cy + 0.5) * 0.05;
      int8_t v = -1;
      for (const auto & a : map.areas) {
        if (!insidePolygon(x, y, a.area.polygon)) continue;
        if (a.type == Area::TYPE_EXCLUSION) {
          v = 100;
          break;
        }
        v = 0;
      }
      g.data[cy * w + cx] = v;
    }
  }
  const auto src = g.data;
  const int r = 5;
  for (int cy = 0; cy < h; ++cy) {
    for (int cx = 0; cx < w; ++cx) {
      if (src[cy * w + cx] == 0) continue;
      for (int dy = -r; dy <= r && g.data[cy * w + cx] != 60; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          const int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h || dx * dx + dy * dy > r * r) continue;
          if (src[ny * w + nx] == 0) {
            g.data[cy * w + cx] = 60;
            break;
          }
        }
      }
    }
  }
  return g;
}

struct World
{
  Map map;
  nav_msgs::msg::OccupancyGrid g;
  FootprintCheck check() const { return FootprintCheck(map, g, Body{}); }
};

// Drives the move like the robot (turn in 5 deg steps, then 5 cm steps):
// nothing on the way may be worse than the start, and the end must fit.
void expectCheckedMove(const FootprintCheck & c, double x, double y, double yaw, const Move & m)
{
  const int start = c.misfit(x, y, yaw);
  const int turn_steps = static_cast<int>(std::ceil(std::abs(m.turn) / (2.5 * M_PI / 180.0)));
  for (int k = 0; k <= turn_steps; ++k) {
    EXPECT_LE(c.misfit(x, y, yaw + m.turn * k / std::max(1, turn_steps)), start) << "turn step " << k;
  }
  const double a = yaw + m.turn;
  const int drive_steps = static_cast<int>(std::ceil(std::abs(m.drive) / 0.01));
  for (int k = 0; k <= drive_steps; ++k) {
    const double d = m.drive * k / std::max(1, drive_steps);
    EXPECT_LE(c.misfit(x + d * std::cos(a), y + d * std::sin(a), a), start) << "drive " << d;
  }
  EXPECT_TRUE(c.fits(x + m.drive * std::cos(a), y + m.drive * std::sin(a), a));
}

World square(double size)
{
  World w;
  w.map.areas.push_back(area({{0, 0}, {size, 0}, {size, size}, {0, size}}, Area::TYPE_OPERATION));
  w.g = grid(w.map, -1.0, -1.0, size + 1.0, size + 1.0);
  return w;
}
}  // namespace

TEST(Footprint, FitsInsideAndOnTheRim)
{
  const auto w = square(4.0);
  const auto c = w.check();
  EXPECT_EQ(c.misfit(2.0, 2.0, 0.3), 0);
  // Centre 5 cm inside the edge, rear 6 cm over it: on the rim.
  EXPECT_TRUE(c.fits(0.05, 2.0, 0.0));
  // Facing out at the edge: the front 0.37 m over it, beyond the 0.25 m rim.
  EXPECT_FALSE(c.fits(3.9, 2.0, 0.0));
  EXPECT_GE(c.misfit(-0.2, 2.0, 0.0), FootprintCheck::kCentreOutside);
}

TEST(Footprint, StraightRoomStopsAtTheRim)
{
  const auto w = square(4.0);
  const auto c = w.check();
  // The body may overhang the edge (rim), the centre never: it reverses to the edge.
  const double room = c.straightRoom(1.0, 2.0, 0.0, -1, 2.0, 0);
  EXPECT_NEAR(room, 1.0, 0.06);
  EXPECT_LT(room, 1.0);
  EXPECT_DOUBLE_EQ(c.straightRoom(2.0, 2.0, 0.0, -1, 0.3, 0), 0.3);
}

TEST(Footprint, ExclusionBehindLimitsTheReverse)
{
  World w;
  w.map.areas.push_back(area({{0, 0}, {6, 0}, {6, 4}, {0, 4}}, Area::TYPE_OPERATION));
  w.map.areas.push_back(area({{1.0, 1.5}, {1.5, 1.5}, {1.5, 2.5}, {1.0, 2.5}}, Area::TYPE_EXCLUSION));
  w.g = grid(w.map, -1.0, -1.0, 7.0, 5.0);
  const auto c = w.check();
  // The rear may overhang the bed by the band (map_server grid.exclusion_band),
  // the centre never enters it: from 2.5 m it reverses to just short of 1.5 m.
  const double room = c.straightRoom(2.5, 2.0, 0.0, -1, 2.0, 0);
  EXPECT_NEAR(room, 1.0, 0.06);
  EXPECT_LT(room, 1.0);
}

TEST(Footprint, EscapeForwardWhenTheRearIsOut)
{
  const auto w = square(4.0);
  const auto c = w.check();
  // Centre 0.2 m outside, facing in: only forward helps; backwards would go further out.
  EXPECT_DOUBLE_EQ(c.straightRoom(-0.2, 2.0, 0.0, -1, 1.0, c.misfit(-0.2, 2.0, 0.0)), 0.0);
  const auto m = c.escape(-0.2, 2.0, 0.0, 1.0);
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->turn, 0.0);
  EXPECT_GT(m->drive, 0.0);
  expectCheckedMove(c, -0.2, 2.0, 0.0, *m);
}

TEST(Footprint, EscapeBackWhenTheFrontIsOut)
{
  const auto w = square(4.0);
  const auto c = w.check();
  const auto m = c.escape(3.9, 2.0, 0.0, 1.0);
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->turn, 0.0);
  EXPECT_NEAR(m->drive, -0.15, 0.051);
  expectCheckedMove(c, 3.9, 2.0, 0.0, *m);
}

TEST(Footprint, NoEscapeFarOutside)
{
  const auto w = square(4.0);
  const auto c = w.check();
  // 1 m beyond the edge: nothing within 0.5 m fits - the robot must not move.
  EXPECT_FALSE(c.escape(-1.0, 2.0, M_PI, 0.5));
}

// The robot 2026-10-09 12:32:14 after Nav2's spin: the old FreeFootprint found
// no straight back-up / turn that fits and reversed 0.30 m blind along -73 deg,
// out of mow_2 (centre 0.23 m outside). Whatever the escape is now, nothing on
// the way is worse and it ends inside line + rim.
TEST(Footprint, Mow2CornerAfterTheSpin)
{
  World w;
  w.map.areas.push_back(area(kMow2, Area::TYPE_OPERATION));
  // map_server also paints the recorded stance at corners; this grid hasn't it.
  // With this cell alignment the pose doesn't fit, as on the robot (by 1 point).
  w.g = grid(w.map, 10.02, -29.0, 19.02, -20.0);
  const auto c = w.check();
  const double x = 14.6336, y = -24.9316, yaw = -1.274;
  EXPECT_GT(c.misfit(x, y, yaw), 0);
  // The blind reverse took the centre out of the area.
  EXPECT_GE(c.misfit(14.54, -24.64, yaw), FootprintCheck::kCentreOutside);
  // Checked: backwards is refused long before that.
  EXPECT_LT(c.straightRoom(x, y, yaw, -1, 0.3, c.misfit(x, y, yaw)), 0.3);
  const auto m = c.escape(x, y, yaw, 1.0);
  if (m) {
    std::cout << "after the spin: turn " << m->turn * 180.0 / M_PI << " deg, drive " << m->drive << " m\n";
    expectCheckedMove(c, x, y, yaw, *m);
  }
  // Where it ended (centre 0.42 m out, facing roughly into the area).
  const double ex = 14.39, ey = -24.524, eyaw = -0.643;
  const auto m2 = c.escape(ex, ey, eyaw, 1.0);
  if (m2) {
    std::cout << "where it ended: turn " << m2->turn * 180.0 / M_PI << " deg, drive " << m2->drive << " m\n";
    expectCheckedMove(c, ex, ey, eyaw, *m2);
  }
}

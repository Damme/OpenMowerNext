// ROS-free tests for the coverage_planner core.
#include "coverage_planner/coverage_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace cp = coverage_planner;

namespace
{
std::vector<cp::Pt> square(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

cp::Result planField(const std::vector<cp::Pt> & outline, std::vector<std::vector<cp::Pt>> holes)
{
  cp::Request req;
  req.outline = outline;
  req.holes = std::move(holes);
  return cp::plan(req, cp::Params{});
}
}  // namespace

TEST(CoveragePlanner, EmptyOutlineGivesNoPaths)
{
  const auto res = planField({}, {});
  EXPECT_TRUE(res.paths.empty());
  EXPECT_TRUE(res.mowable.empty());
}

TEST(CoveragePlanner, PerimeterFirstThenFill)
{
  const auto res = planField(square(0, 0, 10, 6), {});
  ASSERT_GE(res.paths.size(), 2u);
  EXPECT_EQ(res.paths.front().kind, cp::PlannedPath::PERIMETER);
  EXPECT_TRUE(res.paths.front().is_outline);
  bool has_fill = false;
  for (const auto & p : res.paths) {
    EXPECT_GE(p.pts.size(), 2u);
    has_fill |= p.kind == cp::PlannedPath::FILL;
  }
  EXPECT_TRUE(has_fill);
}

TEST(CoveragePlanner, PathsStayInsideOutline)
{
  const auto res = planField(square(0, 0, 10, 6), {square(4, 2.5, 5, 3.5)});
  for (const auto & p : res.paths) {
    for (const auto & pt : p.pts) {
      EXPECT_GE(pt.x, -1e-6);
      EXPECT_LE(pt.x, 10 + 1e-6);
      EXPECT_GE(pt.y, -1e-6);
      EXPECT_LE(pt.y, 6 + 1e-6);
      // never through the obstacle (it is grown by the clearance)
      EXPECT_FALSE(pt.x > 4.01 && pt.x < 4.99 && pt.y > 2.51 && pt.y < 3.49);
    }
  }
}

TEST(CoveragePlanner, InteriorObstacleGetsRing)
{
  const auto res = planField(square(0, 0, 10, 6), {square(4, 2.5, 5, 3.5)});
  bool has_ring = false;
  for (const auto & p : res.paths) {
    has_ring |= p.kind == cp::PlannedPath::OBSTACLE_RING;
  }
  EXPECT_TRUE(has_ring);
}

TEST(CoveragePlanner, OutsideExclusionIsIgnored)
{
  const auto res = planField(square(0, 0, 10, 10), {square(20, 20, 22, 22)});
  ASSERT_EQ(res.mowable.size(), 1u);
  EXPECT_NEAR(bg::area(res.mowable), 100.0, 1e-6);
}

TEST(CoveragePlanner, ExclusionsCutMowableArea)
{
  const auto res = planField(
    square(0, 0, 10, 10), {square(2, 2, 4, 4), square(8, 2, 12, 4), square(20, 20, 22, 22)});
  ASSERT_EQ(res.mowable.size(), 1u);
  EXPECT_NEAR(bg::area(res.mowable), 92.0, 1e-6);
  EXPECT_EQ(res.mowable[0].inners().size(), 1u);
}

TEST(CoveragePlanner, SplittingExclusionGivesTwoComponents)
{
  const auto res = planField(square(0, 0, 10, 10), {square(4, -1, 6, 11)});
  EXPECT_EQ(res.mowable.size(), 2u);
  EXPECT_NEAR(bg::area(res.mowable), 80.0, 1e-6);
}

TEST(CoveragePlanner, HeadingsPointToNextAndKeepLast)
{
  const auto yaw = cp::headings({{0, 0}, {1, 0}, {1, 1}});
  ASSERT_EQ(yaw.size(), 3u);
  EXPECT_NEAR(yaw[0], 0.0, 1e-9);
  EXPECT_NEAR(yaw[1], M_PI / 2, 1e-9);
  EXPECT_NEAR(yaw[2], M_PI / 2, 1e-9);
}

namespace
{
cp::Result planAngle(double deg, bool fixed, bool reverse)
{
  cp::Request req;
  req.outline = square(0, 0, 10, 6);
  req.angle_rad = deg * M_PI / 180.0;
  req.fixed_angle = fixed;
  req.reverse = reverse;
  return cp::plan(req, cp::Params{});  // optimize_sweep_angle is on by default
}

// Fill travel along x and along y, summed over all fill paths.
std::pair<double, double> fillTravel(const cp::Result & res)
{
  double ax = 0.0, ay = 0.0;
  for (const auto & p : res.paths) {
    if (p.kind != cp::PlannedPath::FILL) continue;
    for (size_t i = 1; i < p.pts.size(); ++i) {
      ax += std::fabs(p.pts[i].x - p.pts[i - 1].x);
      ay += std::fabs(p.pts[i].y - p.pts[i - 1].y);
    }
  }
  return {ax, ay};
}

// Net x travel on the southmost fill lane (row 0, the first lane right of the angle 0 direction).
double lowestLaneDx(const cp::Result & res)
{
  double ymin = 1e9;
  for (const auto & p : res.paths)
    if (p.kind == cp::PlannedPath::FILL)
      for (const auto & q : p.pts) ymin = std::min(ymin, q.y);
  double dx = 0.0;
  for (const auto & p : res.paths) {
    if (p.kind != cp::PlannedPath::FILL) continue;
    for (size_t i = 1; i < p.pts.size(); ++i)
      if (p.pts[i].y < ymin + 0.02 && p.pts[i - 1].y < ymin + 0.02) dx += p.pts[i].x - p.pts[i - 1].x;
  }
  return dx;
}

// Fill segments in the middle of the field (away from the U-turns) that `b` drives the other way
// at the same place, as a fraction of `a`'s.
double oppositeFraction(const cp::Result & a, const cp::Result & b)
{
  struct Seg { cp::Pt m, d; };
  auto segs = [](const cp::Result & r) {
    std::vector<Seg> out;
    for (const auto & p : r.paths) {
      if (p.kind != cp::PlannedPath::FILL) continue;
      for (size_t i = 1; i < p.pts.size(); ++i) {
        const cp::Pt m((p.pts[i].x + p.pts[i - 1].x) / 2, (p.pts[i].y + p.pts[i - 1].y) / 2);
        if (m.x > 3.0 && m.x < 7.0 && m.y > 1.5 && m.y < 4.5)
          out.push_back({m, cp::Pt(p.pts[i].x - p.pts[i - 1].x, p.pts[i].y - p.pts[i - 1].y)});
      }
    }
    return out;
  };
  const auto sa = segs(a), sb = segs(b);
  int n = 0, opposite = 0;
  for (const auto & s : sa) {
    const Seg * best = nullptr;
    double bd = 0.06;
    for (const auto & t : sb) {
      const double d = std::hypot(t.m.x - s.m.x, t.m.y - s.m.y);
      if (d < bd) { bd = d; best = &t; }
    }
    ++n;
    if (best && best->d.x * s.d.x + best->d.y * s.d.y < 0.0) ++opposite;
  }
  return n ? static_cast<double>(opposite) / n : 0.0;
}
}  // namespace

TEST(CoveragePlanner, AreaAngleOverridesOptimizedAngle)
{
  // The long axis is x; without fixed_angle the 90 deg request is ignored.
  const auto [ox, oy] = fillTravel(planAngle(90.0, false, false));
  EXPECT_GT(ox, 3.0 * oy);
  const auto [fx, fy] = fillTravel(planAngle(90.0, true, false));
  EXPECT_GT(fy, 3.0 * fx);
}

TEST(CoveragePlanner, ReverseDrivesTheSameLanesTheOtherWay)
{
  const auto fwd = planAngle(0.0, true, false), rev = planAngle(0.0, true, true);
  EXPECT_GT(lowestLaneDx(fwd), 5.0);  // the first lane (right of the direction) runs along it
  EXPECT_LT(lowestLaneDx(rev), -5.0);
  EXPECT_GT(oppositeFraction(fwd, rev), 0.95);
  // The automatic angle reverses too.
  EXPECT_GT(oppositeFraction(planAngle(0.0, false, false), planAngle(0.0, false, true)), 0.95);
}

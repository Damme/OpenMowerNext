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

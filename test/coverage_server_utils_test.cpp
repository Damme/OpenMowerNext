#include "coverage_server/utils.h"

#include <gtest/gtest.h>

namespace utils = open_mower_next::coverage_server::utils;

namespace
{
geometry_msgs::msg::Polygon makePolygon(std::initializer_list<std::pair<double, double>> points)
{
  geometry_msgs::msg::Polygon polygon;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    polygon.points.push_back(point);
  }
  return polygon;
}

coverage_planner::BMultiPolygon mowable(
  const geometry_msgs::msg::Polygon & field,
  const std::vector<geometry_msgs::msg::Polygon> & exclusions)
{
  coverage_planner::Request req;
  req.outline = utils::toPoints(field);
  for (const auto & e : exclusions) {
    req.holes.push_back(utils::toPoints(e));
  }
  return coverage_planner::plan(req, coverage_planner::Params{}).mowable;
}
}  // namespace

TEST(CoverageServerUtils, EmptyPolygonIsInvalid)
{
  EXPECT_FALSE(utils::isValid(geometry_msgs::msg::Polygon{}));
  EXPECT_TRUE(utils::isValid(makePolygon({{0, 0}, {1, 0}, {1, 1}})));
}

TEST(CoverageServerUtils, IntersectsDetectsOutsideExclusion)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  EXPECT_FALSE(utils::intersects(field, makePolygon({{20, 20}, {22, 20}, {22, 22}, {20, 22}})));
  EXPECT_TRUE(utils::intersects(field, makePolygon({{8, 2}, {12, 2}, {12, 4}, {8, 4}})));
  EXPECT_TRUE(utils::intersects(field, makePolygon({{2, 2}, {4, 2}, {4, 4}, {2, 4}})));
}

TEST(CoverageServerUtils, InsideExclusionConvertsToCoverageGeometryHole)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto inside_exclusion = makePolygon({{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}});

  const auto msg = utils::toMsg(mowable(field, {inside_exclusion}), "map");

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.cells[0].header.frame_id, "map");
  EXPECT_EQ(msg.cells[0].exterior.points.size(), 4u);
  ASSERT_EQ(msg.cells[0].holes.size(), 1u);
  EXPECT_EQ(msg.cells[0].holes[0].points.size(), 4u);
}

TEST(CoverageServerUtils, PartialOverlapConvertsToClippedCoverageGeometry)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto overlapping_exclusion = makePolygon({{8.0, 2.0}, {12.0, 2.0}, {12.0, 4.0}, {8.0, 4.0}});

  const auto msg = utils::toMsg(mowable(field, {overlapping_exclusion}), "map");

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_TRUE(msg.cells[0].holes.empty());
  EXPECT_GE(msg.cells[0].exterior.points.size(), 4u);
}

TEST(CoverageServerUtils, SplittingExclusionConvertsToMultipleCoverageCells)
{
  const auto field = makePolygon({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
  const auto splitting_exclusion = makePolygon({{4.0, -1.0}, {6.0, -1.0}, {6.0, 11.0}, {4.0, 11.0}});

  const auto msg = utils::toMsg(mowable(field, {splitting_exclusion}), "map");

  ASSERT_EQ(msg.cells.size(), 2u);
  EXPECT_TRUE(msg.cells[0].holes.empty());
  EXPECT_TRUE(msg.cells[1].holes.empty());
}

TEST(CoverageServerUtils, PlannedPathConvertsToNavPathWithYaw)
{
  coverage_planner::PlannedPath planned;
  planned.kind = coverage_planner::PlannedPath::PERIMETER;
  planned.is_outline = true;
  planned.pts = {{0, 0}, {1, 0}, {1, 1}};
  std_msgs::msg::Header header;
  header.frame_id = "map";

  const auto msg = utils::toMsg(planned, header);

  EXPECT_EQ(msg.kind, open_mower_next::msg::CoveragePath::KIND_PERIMETER);
  EXPECT_TRUE(msg.is_outline);
  ASSERT_EQ(msg.path.poses.size(), 3u);
  EXPECT_EQ(msg.path.header.frame_id, "map");
  EXPECT_NEAR(msg.path.poses[1].pose.orientation.z, std::sin(M_PI / 4), 1e-6);
  EXPECT_NEAR(msg.path.poses[2].pose.orientation.z, std::sin(M_PI / 4), 1e-6);

  const auto all = utils::concatenate({msg, msg}, header);
  EXPECT_EQ(all.poses.size(), 6u);
}

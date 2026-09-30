#include "mower_logic/felt_obstacles.hpp"
#include "mower_logic/mission.hpp"
#include "mower_logic/mowed_area.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

using open_mower_next::mower_logic::Contact;
using open_mower_next::mower_logic::FeltObstacle;
using open_mower_next::mower_logic::MarkShape;
using open_mower_next::mower_logic::Mission;
using open_mower_next::mower_logic::MowedArea;
using open_mower_next::msg::CoveragePath;

namespace
{
CoveragePath straight(int n, double step = 0.1)
{
  CoveragePath p;
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = i * step;
    p.path.poses.push_back(ps);
  }
  return p;
}
}  // namespace

TEST(Mission, WalksAreasAndPasses)
{
  Mission m;
  m.begin({"a", "b"});
  ASSERT_TRUE(m.active());
  EXPECT_EQ(m.areaNeedingPlan(), "a");
  EXPECT_FALSE(m.currentPass());
  m.setPlan({straight(10), straight(20)});
  EXPECT_FALSE(m.areaNeedingPlan());
  auto p = m.currentPass(0.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->path.poses.size(), 10u);
  m.passDone();
  EXPECT_EQ(m.currentPass(0.0)->path.poses.size(), 20u);
  m.passDone();
  EXPECT_EQ(m.areaNeedingPlan(), "b");  // next area
  m.setPlan({straight(5)});
  m.passDone();
  EXPECT_FALSE(m.active());  // done
  EXPECT_FALSE(m.currentPass());
}

TEST(Mission, ResumesWithBacktrack)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1)});
  m.setPoseIndex(50);
  m.setPoseIndex(30);  // progress never goes backwards
  EXPECT_EQ(m.poseIndex(), 50u);
  auto p = m.currentPass(0.5);  // 0.5 m = 5 poses back
  ASSERT_TRUE(p);
  EXPECT_EQ(p->start_index, 45u);
  EXPECT_EQ(p->path.poses.size(), 55u);
  EXPECT_NEAR(p->path.poses.front().pose.position.x, 4.5, 1e-9);
}

TEST(Mission, ReportsMetresCovered)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1)});
  EXPECT_NEAR(m.setPoseIndex(20), 2.0, 1e-9);
  EXPECT_NEAR(m.setPoseIndex(15), 0.0, 1e-9);  // backtrack: already counted
  EXPECT_NEAR(m.setPoseIndex(25), 0.5, 1e-9);
  m.continueAt(60);  // skipped past an obstacle: not mowed
  EXPECT_NEAR(m.setPoseIndex(61), 0.1, 1e-9);
  EXPECT_NEAR(m.setPoseIndex(500), 3.8, 1e-9);  // clamped to the pass end
}

TEST(MowedArea, RollsChargesAndPersists)
{
  MowedArea a;
  a.docked();  // nothing mowed: no roll
  EXPECT_EQ(a.rolls(), 0u);
  EXPECT_TRUE(a.history().empty());
  for (double m2 : {10.0, 20.0, 30.0, 40.0}) {
    a.add(m2);
    a.docked();
  }
  a.add(5.0);
  a.add(-1.0);  // ignored
  EXPECT_DOUBLE_EQ(a.charge(), 5.0);
  EXPECT_DOUBLE_EQ(a.total(), 105.0);
  EXPECT_EQ(a.history(), (std::deque<double>{40.0, 30.0, 20.0}));  // newest first, 3 kept
  EXPECT_EQ(a.json(), "{\"charge\":5,\"last\":[40,30,20],\"total\":105}");

  MowedArea b;
  ASSERT_TRUE(b.restore(a.serialize()));
  EXPECT_EQ(b.json(), a.json());
  EXPECT_FALSE(b.restore("garbage"));
  EXPECT_EQ(b.json(), a.json());
}

TEST(Mission, SkipsPassAfterRepeatedFailures)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(10), straight(10)});
  EXPECT_FALSE(m.passFailed(3));
  EXPECT_FALSE(m.passFailed(3));
  EXPECT_TRUE(m.passFailed(3));  // third failure skips
  auto p = m.currentPass(0.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->pass_index, 1u);
}

TEST(Mission, SkipAreaEndsMission)
{
  Mission m;
  m.begin({"a"});
  m.skipArea();
  EXPECT_FALSE(m.active());
  EXPECT_EQ(m.summary(), "no mission");
}

TEST(Mission, SkipsPastBumpWithoutBacktrack)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1), straight(10)});
  m.setPoseIndex(30);  // robot at x = 3.0, obstacle 0.55 m ahead
  ASSERT_TRUE(m.skipPastPoint(3.55, 0.0, 0.7, 4));
  auto p = m.currentPass(0.5);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->start_index, 43u);  // first pose > 0.7 m past the obstacle (x = 4.3), no backtrack
  EXPECT_NEAR(p->path.poses.front().pose.position.x, 4.3, 1e-9);
  m.setPoseIndex(50);  // progress again: normal backtrack
  EXPECT_EQ(m.currentPass(0.5)->start_index, 45u);
}

TEST(Mission, BumpAtPassEndOrTooOftenSkipsPass)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1), straight(10)});
  m.setPoseIndex(95);
  EXPECT_FALSE(m.skipPastPoint(9.8, 0.0, 0.7, 4));  // nothing left past the obstacle
  EXPECT_EQ(m.currentPass(0.0)->path.poses.size(), 10u);  // next pass

  Mission m2;
  m2.begin({"a"});
  m2.setPlan({straight(100, 0.1), straight(10)});
  EXPECT_TRUE(m2.skipPastPoint(1.0, 0.0, 0.3, 2));
  EXPECT_TRUE(m2.skipPastPoint(2.0, 0.0, 0.3, 2));
  EXPECT_FALSE(m2.skipPastPoint(3.0, 0.0, 0.3, 2));  // third bump on the pass
  EXPECT_EQ(m2.currentPass(0.0)->path.poses.size(), 10u);
}

TEST(Mission, FailedContinuationSkipsFurther)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1)});
  m.setPoseIndex(30);
  ASSERT_TRUE(m.skipPastPoint(3.55, 0.0, 0.7, 4));  // continue at 43, no backtrack
  EXPECT_FALSE(m.passFailed(3, 1.0, 3));            // unreachable: 1 m further
  EXPECT_EQ(m.currentPass(0.5)->start_index, 53u);
  EXPECT_FALSE(m.passFailed(3, 1.0, 3));            // then 2 m
  EXPECT_EQ(m.currentPass(0.5)->start_index, 73u);
  EXPECT_FALSE(m.passFailed(3, 1.0, 3));            // then 4 m: hits the pass end, counts as attempt 1
  EXPECT_EQ(m.currentPass(0.5)->start_index, 97u);
  EXPECT_FALSE(m.passFailed(3, 1.0, 3));            // steps used up: ordinary attempts
  EXPECT_TRUE(m.passFailed(3, 1.0, 3));             // third failure: pass skipped
}

TEST(Mission, OperatorSkipsChangeGeneration)
{
  Mission m;
  m.begin({"a", "b"});
  const auto g0 = m.generation();
  m.setPlan({straight(10), straight(10)});
  m.passDone();  // normal progress: running actions stay valid
  EXPECT_EQ(m.generation(), g0);
  m.skipPass();  // operator skip: a running pass/transit is stale
  EXPECT_NE(m.generation(), g0);
  const auto g1 = m.generation();
  m.skipArea();
  EXPECT_NE(m.generation(), g1);
}

TEST(Mission, SavedPositionSurvivesARestart)
{
  Mission m;
  m.begin({"a", "b"});
  m.setPlan({straight(10), straight(100), straight(20)});
  m.passDone();
  m.setPoseIndex(42);
  const auto saved = m.serialize();
  ASSERT_FALSE(saved.empty());

  Mission r;  // after a restart
  ASSERT_TRUE(r.restore(saved));
  EXPECT_TRUE(r.active());
  EXPECT_EQ(r.areaNeedingPlan(), "a");       // re-planned on demand
  EXPECT_EQ(r.serialize(), saved);           // still saved before the re-plan
  const auto note = r.setPlan({straight(10), straight(100), straight(20)});
  EXPECT_NE(note.find("restored"), std::string::npos);
  EXPECT_EQ(r.poseIndex(), 42u);
  auto p = r.currentPass(0.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->pass_index, 1u);
  EXPECT_EQ(p->start_index, 42u);
}

TEST(Mission, ChangedPlanDiscardsTheSavedPosition)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(10), straight(100)});
  m.passDone();
  m.setPoseIndex(42);
  Mission r;
  ASSERT_TRUE(r.restore(m.serialize()));
  const auto note = r.setPlan({straight(10), straight(90)});  // map / planner changed
  EXPECT_NE(note.find("discarded"), std::string::npos);
  auto p = r.currentPass(0.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->pass_index, 0u);
  EXPECT_EQ(p->start_index, 0u);
}

TEST(Mission, NothingSavedWithoutAMissionAndBadTextIgnored)
{
  Mission m;
  EXPECT_TRUE(m.serialize().empty());
  m.begin({"a"});
  m.clear();  // reset_mission
  EXPECT_TRUE(m.serialize().empty());
  EXPECT_FALSE(m.restore(""));
  EXPECT_FALSE(m.restore("openmower_mission 1\nareas a\narea 3\n"));
  EXPECT_FALSE(m.restore("something else\n"));
  EXPECT_FALSE(m.active());
}

TEST(Mission, SkipPastAnyShapeAndCountBumps)
{
  Mission m;
  m.begin({"a"});
  m.setPlan({straight(100, 0.1), straight(10)});
  m.setPoseIndex(20);
  // Obstacle covering x in [3.0, 3.5] of the pass.
  auto near = [](double x, double) { return x >= 3.0 && x <= 3.5; };
  ASSERT_TRUE(m.skipPast(near, 4));
  EXPECT_EQ(m.currentPass(0.5)->start_index, 36u);  // first pose after it, no backtrack
  // Nothing of the rest is near: the pass is done (as for an obstacle at the end).
  EXPECT_FALSE(m.skipPast([](double, double) { return false; }, 4));
  EXPECT_EQ(m.currentPass(0.0)->path.poses.size(), 10u);

  Mission m2;
  m2.begin({"a"});
  m2.setPlan({straight(100, 0.1), straight(10)});
  EXPECT_TRUE(m2.countBump(2));
  EXPECT_TRUE(m2.countBump(2));
  EXPECT_FALSE(m2.countBump(2));  // third bump: pass skipped
  EXPECT_EQ(m2.currentPass(0.0)->path.poses.size(), 10u);
}

TEST(FeltObstacles, ContactMarksAThinBandAtTheFront)
{
  MarkShape s;  // front 0.47, half width 0.195, chamfer 0.10, gap 0.02, depth 0.10
  Contact c;
  c.x = 1.0;
  c.y = 2.0;
  c.yaw = M_PI / 2;  // facing +y
  c.marks = open_mower_next::mower_logic::contactMarks(c, s);
  ASSERT_FALSE(c.marks.empty());
  double min_y = 1e9, max_y = -1e9, min_x = 1e9, max_x = -1e9;
  for (const auto & [x, y] : c.marks) {
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }
  // Nothing inside the body; the front face band is 0.49 .. 0.59 ahead.
  EXPECT_GE(min_y, 2.0 + 0.47 - 0.10 - 0.05 - 1e-9);  // sides behind the cut corners reach back 5 cm
  EXPECT_NEAR(max_y, 2.0 + 0.47 + 0.12, 1e-9);
  EXPECT_NEAR(max_x - min_x, 2 * (0.195 + 0.12), 1e-9);  // both sides + band
  FeltObstacle o{1, {c}};
  EXPECT_NEAR(o.distance(1.0, 2.0 + 0.49), 0.0, 1e-9);   // on the band
  EXPECT_NEAR(o.distance(1.0, 2.0 + 0.30), 0.19, 1e-9);  // 0.3 m ahead of the robot centre
  // No mark inside the footprint (the robot stood there): convex hexagon, counter-clockwise.
  const double fp[][2] = {{0.47, 0.095}, {0.37, 0.195}, {-0.11, 0.195}, {-0.11, -0.195}, {0.37, -0.195}, {0.47, -0.095}};
  for (const auto & [x, y] : c.marks) {
    const double u = y - 2.0, v = -(x - 1.0);  // robot frame (facing +y)
    bool inside = true;
    for (int i = 0; i < 6; ++i) {
      const auto & a = fp[i];
      const auto & b = fp[(i + 1) % 6];
      if ((b[0] - a[0]) * (v - a[1]) - (b[1] - a[1]) * (u - a[0]) < 0.01) inside = false;  // 1 cm tolerance
    }
    EXPECT_FALSE(inside) << u << " " << v;
  }
}

TEST(FeltObstacles, SideContactOnlyMarksThatCorner)
{
  MarkShape s;
  Contact left;
  left.side = 1;  // facing +x, left = +y
  left.marks = open_mower_next::mower_logic::contactMarks(left, s);
  Contact right = left;
  right.side = -1;
  right.marks = open_mower_next::mower_logic::contactMarks(right, s);
  // The cut corner and the outer half of that side's front face (half width 0.195, chamfer 0.10).
  for (const auto & [x, y] : left.marks) EXPECT_GE(y, 0.0475 - 1e-9);
  for (const auto & [x, y] : right.marks) EXPECT_LE(y, -0.0475 + 1e-9);
  EXPECT_LT(left.marks.size(), open_mower_next::mower_logic::contactMarks(Contact{}, s).size() / 2);
  Contact again = left;
  again.x += 0.02;
  EXPECT_TRUE(open_mower_next::mower_logic::sameContact(left, again));
  again.side = 0;
  EXPECT_FALSE(open_mower_next::mower_logic::sameContact(left, again));
}

TEST(FeltObstacles, JogShiftsAroundThePoseAndFadesOut)
{
  using open_mower_next::mower_logic::PathPose;
  std::vector<PathPose> poses;
  for (int i = 0; i < 100; ++i) poses.push_back({i * 0.05, 0.0, 0.0});  // 5 m along +x
  const auto [first, last] = open_mower_next::mower_logic::jogPath(poses, 0, 50, 0.15, 0.35, 0.5);
  ASSERT_LE(first, last);
  EXPECT_NEAR(poses[50].y, 0.15, 1e-9);          // fully shifted at the spot (x = 2.5)
  EXPECT_NEAR(poses[44].y, 0.15, 1e-9);          // 0.3 m before it too
  EXPECT_NEAR(poses[10].y, 0.0, 1e-9);           // far before: unchanged
  EXPECT_NEAR(poses[90].y, 0.0, 1e-9);           // far after: unchanged
  EXPECT_GT(poses[36].y, 0.0);                   // in the ramp: partly
  EXPECT_LT(poses[36].y, 0.15);
  EXPECT_GT(poses[37].yaw, 0.0);                 // heading follows the ramp up
  EXPECT_LT(poses[63].yaw, 0.0);                 // and down
  EXPECT_NEAR(static_cast<double>(first), 34.0, 1.0);  // 0.85 m before (the fade's edge, rounding)
  EXPECT_NEAR(static_cast<double>(last), 66.0, 1.0);
}

TEST(FeltObstacles, DrivingThroughErasesMarks)
{
  MarkShape s;
  Contact c;  // at the origin facing +x
  c.marks = open_mower_next::mower_logic::contactMarks(c, s);
  c.total = c.marks.size();
  FeltObstacle o{1, {c}};
  // The robot 0.3 m further on the same line: its body covers the front part of the band.
  const size_t n = o.erase([&](double x, double y) {
    return open_mower_next::mower_logic::insideBody(s.body, 0.3, 0.0, 0.0, x, y);
  });
  EXPECT_GT(n, 0u);
  EXPECT_LT(o.markCount(), c.total);
  EXPECT_GT(o.markCount(), 0u);  // the parts beside the body stay
  EXPECT_TRUE(open_mower_next::mower_logic::insideBody(s.body, 0.0, 0.0, 0.0, 0.46, 0.0));
  EXPECT_FALSE(open_mower_next::mower_logic::insideBody(s.body, 0.0, 0.0, 0.0, 0.46, 0.18));  // cut corner
  EXPECT_FALSE(open_mower_next::mower_logic::insideBody(s.body, 0.0, 0.0, 0.0, -0.12, 0.0));
}

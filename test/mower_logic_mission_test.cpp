#include "mower_logic/mission.hpp"

#include <gtest/gtest.h>

using open_mower_next::mower_logic::Mission;
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

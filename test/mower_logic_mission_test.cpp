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

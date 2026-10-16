#include <gtest/gtest.h>
#include "ego_ground_sim/scan_guard.hpp"
using ego_ground_sim::sweepBlocked;

TEST(ScanGuard, StraightApproachAndReverse)
{
  EXPECT_TRUE(sweepBlocked({{0.5, 0}}, 0.22, 0, 0.30, 0.8, 0.4));
  EXPECT_FALSE(sweepBlocked({{0.5, 0}}, -0.22, 0, 0.30, 0.8, 0.4));
  EXPECT_TRUE(sweepBlocked({{-0.5, 0}}, -0.22, 0, 0.30, 0.8, 0.4));
  EXPECT_FALSE(sweepBlocked({{0.8, 0}}, 0.22, 0, 0.30, 0.8, 0.4));
}

TEST(ScanGuard, TurningArcAndSideClearance)
{
  EXPECT_TRUE(sweepBlocked({{0.2, 0.36}}, 0.22, 0.8, 0.30, 0.8, 0.4));
  EXPECT_FALSE(sweepBlocked({{0.2, -0.36}}, 0.22, 0.8, 0.30, 0.8, 0.4));
  EXPECT_FALSE(sweepBlocked({{0.2, 0.5}, {0.2, -0.5}}, 0.22, 0, 0.30, 0.8, 0.4));
}

TEST(ScanGuard, RotationUsesFullBodyEnvelope)
{
  EXPECT_TRUE(sweepBlocked({{0, 0.25}}, 0, 0.8, 0.30, 0.8, 0.4));
  EXPECT_FALSE(sweepBlocked({{0, 0.4}}, 0, 0.8, 0.30, 0.8, 0.4));
}

TEST(ScanGuard, SoftOverlapAllowsOnlyOutwardMotion)
{
  using ego_ground_sim::guardMotionBlocked;
  EXPECT_FALSE(guardMotionBlocked({{-0.26, 0}}, 0.06, 0, 0.21, 0.30, 0.8, 0.4));
  EXPECT_TRUE(guardMotionBlocked({{0.26, 0}}, 0.06, 0, 0.21, 0.30, 0.8, 0.4));
  EXPECT_FALSE(guardMotionBlocked({{0.26, 0}}, 0, 0.5, 0.21, 0.30, 0.8, 0.4));
  EXPECT_TRUE(guardMotionBlocked({{0.20, 0}}, 0, 0.5, 0.21, 0.30, 0.8, 0.4));
  EXPECT_TRUE(guardMotionBlocked({{0, 0.26}}, 0.06, 0.8, 0.21, 0.30, 0.8, 0.4));
}

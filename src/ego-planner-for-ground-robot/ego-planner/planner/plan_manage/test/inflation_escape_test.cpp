#include <gtest/gtest.h>
#include <plan_manage/inflation_escape.hpp>
using namespace ego_planner;

TEST(InflationEscape, FindsOutwardExitWithObstacleBehindHeading)
{
  EscapePoints wall;
  for (double y = -1; y <= 1; y += 0.025) {wall.emplace_back(0, y);}
  Eigen::Vector2d target;
  ASSERT_TRUE(findInflationExit({0.28, 0}, M_PI, {2, 0}, wall,
    [](const auto & p) {return p.x() > 0.45;}, 0.21, target));
  EXPECT_GT(target.x(), 0.45);
  const auto command = inflationExitCommand({0.28, 0}, M_PI, target, 0.06, 0.5);
  EXPECT_DOUBLE_EQ(command.x(), 0.0);
  EXPECT_GT(std::abs(command.y()), 0.4);
  const auto aligned = inflationExitCommand({0.28, 0}, 0, target, 0.06, 0.5);
  EXPECT_GE(aligned.x(), 0.0);
  EXPECT_LE(aligned.x(), 0.06);
}

TEST(InflationEscape, EscapesCornerWithoutCrossingRealWalls)
{
  EscapePoints walls;
  for (double a = -1; a <= 1; a += 0.025) {
    walls.emplace_back(0, a); walls.emplace_back(a, 0);
  }
  Eigen::Vector2d target;
  ASSERT_TRUE(findInflationExit({0.28, 0.28}, 0, {2, 2}, walls,
    [](const auto & p) {return p.x() > 0.45 && p.y() > 0.45;}, 0.21, target));
  EXPECT_GT(target.x(), 0.45); EXPECT_GT(target.y(), 0.45);
  EXPECT_TRUE(escapeSegmentClear({0.28, 0.28}, target, walls, 0.21));
}

TEST(InflationEscape, RefusesPhysicalOverlapOrNoExit)
{
  EscapePoints wall = {{0, 0}, {0, 0.1}, {0, -0.1}};
  Eigen::Vector2d target;
  EXPECT_FALSE(findInflationExit({0.15, 0}, 0, {2, 0}, wall,
    [](const auto &) {return true;}, 0.21, target));
  EXPECT_FALSE(findInflationExit({0.28, 0}, 0, {2, 0}, wall,
    [](const auto &) {return false;}, 0.21, target));
}

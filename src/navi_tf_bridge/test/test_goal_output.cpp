#include <gtest/gtest.h>

#include "navi_tf_bridge/goal_output.hpp"

namespace navi_tf_bridge
{

TEST(GoalOutput, UniformScaleOnePreservesMapPoint)
{
  geometry_msgs::msg::Point point;
  point.x = 12.5;
  point.y = -7.25;
  point.z = 0.4;

  const auto scaled = ScaleGoalPosePoint(point, 1.0);

  EXPECT_DOUBLE_EQ(scaled.x, 12.5);
  EXPECT_DOUBLE_EQ(scaled.y, -7.25);
  EXPECT_DOUBLE_EQ(scaled.z, 0.4);
}

TEST(GoalOutput, UniformScaleChangesOnlyPlanarGoalPosition)
{
  geometry_msgs::msg::Point point;
  point.x = 12.5;
  point.y = -7.25;
  point.z = 0.4;

  const auto scaled = ScaleGoalPosePoint(point, 0.8);

  EXPECT_DOUBLE_EQ(scaled.x, 10.0);
  EXPECT_DOUBLE_EQ(scaled.y, -5.8);
  EXPECT_DOUBLE_EQ(scaled.z, 0.4);
}

}  // namespace navi_tf_bridge

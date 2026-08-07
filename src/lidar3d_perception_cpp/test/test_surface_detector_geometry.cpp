#include <gtest/gtest.h>

#include "lidar3d_perception_cpp/surface_detector.hpp"

namespace
{

lidar3d::Cloud makeGround(double longitudinal_grade)
{
  lidar3d::Cloud ground;
  for (double x = 1.0; x <= 8.0; x += 0.2) {
    for (double y = -2.0; y <= 2.0; y += 0.2) {
      ground.emplace_back(x, y, longitudinal_grade * x);
    }
  }
  return ground;
}

TEST(SurfaceDetectorGeometry, IgnoresLevelGround)
{
  const auto patches = lidar3d::detectSlopePatches(makeGround(0.0));
  EXPECT_TRUE(patches.empty());
}

TEST(SurfaceDetectorGeometry, ReportsApexAndLongitudinalSpanForGroundSlope)
{
  const auto patches = lidar3d::detectSlopePatches(makeGround(0.08));
  ASSERT_FALSE(patches.empty());

  const auto & patch = patches.front();
  EXPECT_GT(patch.apex.x(), 6.5);
  EXPECT_GT(patch.dims.x(), 5.0);
  EXPECT_GT(patch.max_grade_deg, 3.0);
  EXPECT_LT(patch.max_grade_deg, 8.0);
}

}  // namespace

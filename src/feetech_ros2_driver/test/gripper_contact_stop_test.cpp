#include <gtest/gtest.h>

#include <feetech_ros2_driver/gripper_contact_stop.hpp>

namespace feetech_ros2_driver {

TEST(GripperContactStop, RequiresConsecutiveStalledSamples) {
  GripperContactStop stop({true, 100, 3, 0.01, 0.02});

  EXPECT_FALSE(stop.update(1.0, 2.0, 120, 0.0).has_value());
  EXPECT_FALSE(stop.update(1.0, 2.0, 120, 0.0).has_value());
  const auto held = stop.update(1.0, 2.0, 120, 0.0);

  ASSERT_TRUE(held.has_value());
  EXPECT_DOUBLE_EQ(*held, 1.0);
  EXPECT_TRUE(stop.latched());
}

TEST(GripperContactStop, IgnoresCurrentWhileTheJawMoves) {
  GripperContactStop stop({true, 100, 2, 0.01, 0.02});

  EXPECT_FALSE(stop.update(1.0, 2.0, 500, 0.1).has_value());
  EXPECT_FALSE(stop.update(1.1, 2.0, 500, 0.1).has_value());
  EXPECT_FALSE(stop.latched());
}

TEST(GripperContactStop, OpeningReleasesTheLatch) {
  GripperContactStop stop({true, 100, 1, 0.01, 0.02});
  ASSERT_TRUE(stop.update(1.0, 2.0, 120, 0.0).has_value());

  EXPECT_FALSE(stop.update(1.0, 0.5, 120, 0.0).has_value());
  EXPECT_FALSE(stop.latched());
}

}  // namespace feetech_ros2_driver

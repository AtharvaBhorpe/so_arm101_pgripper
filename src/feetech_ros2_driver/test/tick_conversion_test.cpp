#include <gtest/gtest.h>

#include <feetech_ros2_driver/tick_conversion.hpp>

namespace feetech_ros2_driver {

TEST(TickConversion, CenterIsZeroInBothDirections) {
  EXPECT_DOUBLE_EQ(ticks_to_radians(2048, 1), 0.0);
  EXPECT_DOUBLE_EQ(ticks_to_radians(2048, -1), 0.0);
}

TEST(TickConversion, DecreasingTicksCanMeanPositiveMotion) {
  EXPECT_NEAR(ticks_to_radians(701, -1), 2.066272, 1e-6);
  EXPECT_EQ(radians_to_ticks(2.0664, -1), 701);
}

TEST(TickConversion, RejectsInvalidDirection) {
  EXPECT_THROW(ticks_to_radians(2048, 0), std::invalid_argument);
}

}  // namespace feetech_ros2_driver

#include <gtest/gtest.h>

#include <so_arm101_pgripper_hardware/tick_conversion.hpp>

namespace so_arm101_pgripper_hardware {

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

}  // namespace so_arm101_pgripper_hardware

#pragma once

#include <feetech_driver/common.hpp>

#include <stdexcept>

namespace feetech_ros2_driver {

inline double ticks_to_radians(const int ticks, const int direction, const int center_tick = 2048) {
  if (direction != -1 && direction != 1) {
    throw std::invalid_argument("direction must be -1 or 1");
  }
  return feetech_driver::to_radians(direction * (ticks - center_tick));
}

inline int radians_to_ticks(const double radians, const int direction, const int center_tick = 2048) {
  if (direction != -1 && direction != 1) {
    throw std::invalid_argument("direction must be -1 or 1");
  }
  return center_tick + direction * feetech_driver::from_radians(radians);
}

}  // namespace feetech_ros2_driver

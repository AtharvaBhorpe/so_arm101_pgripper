#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>

namespace feetech_ros2_driver {

struct GripperContactStopConfig {
  bool enabled{false};
  int current_threshold{0};
  std::size_t stop_cycles{5};
  double position_epsilon{0.001};
  double release_epsilon{0.002};
};

class GripperContactStop {
 public:
  explicit GripperContactStop(GripperContactStopConfig config) : config_(config) {}

  std::optional<double> update(const double measured_position,
                               const double commanded_position,
                               const int current,
                               const double position_delta) {
    if (!config_.enabled) {
      reset();
      return std::nullopt;
    }
    if (latched_) {
      if (commanded_position < measured_position - config_.release_epsilon) {
        reset();
        return std::nullopt;
      }
      return measured_position;
    }
    if (commanded_position <= measured_position + config_.position_epsilon) {
      consecutive_contact_cycles_ = 0;
      return std::nullopt;
    }
    if (current >= config_.current_threshold && position_delta <= config_.position_epsilon) {
      ++consecutive_contact_cycles_;
    } else {
      consecutive_contact_cycles_ = 0;
    }
    if (consecutive_contact_cycles_ >= std::max<std::size_t>(1, config_.stop_cycles)) {
      latched_ = true;
      consecutive_contact_cycles_ = 0;
      return measured_position;
    }
    return std::nullopt;
  }

  [[nodiscard]] bool latched() const { return latched_; }

  void reset() {
    latched_ = false;
    consecutive_contact_cycles_ = 0;
  }

 private:
  GripperContactStopConfig config_;
  std::size_t consecutive_contact_cycles_{0};
  bool latched_{false};
};

}  // namespace feetech_ros2_driver

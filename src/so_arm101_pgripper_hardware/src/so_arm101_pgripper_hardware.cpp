#include <fmt/ranges.h>

#include <algorithm>
#include <array>
#include <feetech_driver/SMS_STS.h>
#include <feetech_driver/common.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_ros2_driver/joint_config.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "so_arm101_pgripper_hardware/so_arm101_pgripper_hardware.hpp"
#include "so_arm101_pgripper_hardware/tick_conversion.hpp"

namespace so_arm101_pgripper_hardware {
namespace {

using feetech_ros2_driver::JointIdConfigMap;
using feetech_ros2_driver::JointParams;

CallbackReturn load_yaml_config(const hardware_interface::HardwareInfo& info, JointIdConfigMap& out_yaml) {
  out_yaml.clear();
  const auto cfg_it = info.hardware_parameters.find("joint_config_file");
  if (cfg_it == info.hardware_parameters.end() || cfg_it->second.empty()) {
    return CallbackReturn::SUCCESS;
  }

  auto loaded = feetech_ros2_driver::load_joint_config(cfg_it->second);
  if (!loaded) {
    return CallbackReturn::ERROR;
  }
  for (auto& [name, params] : *loaded) {
    const auto id_it = params.find("id");
    if (id_it == params.end() || !out_yaml.emplace(std::stoi(id_it->second), std::move(params)).second) {
      spdlog::error("YAML joint '{}' has a missing or duplicate id", name);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace

#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn SoArm101PgripperHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn SoArm101PgripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error("SO-ARM101 hardware plugin needs the usb_port parameter");
    return CallbackReturn::ERROR;
  }
  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);
  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("SO-ARM101 hardware transport -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  JointIdConfigMap yaml_by_id;
  if (load_yaml_config(info_, yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_ids_.assign(info_.joints.size(), 0);
  joint_center_ticks_.assign(info_.joints.size(), feetech_driver::kStsMidpoint);
  joint_directions_.assign(info_.joints.size(), 1);
  state_hw_currents_.assign(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const auto id_it = joint.parameters.find("id");
    if (id_it == joint.parameters.end()) {
      spdlog::error("Joint '{}' does not have required id", joint.name);
      return CallbackReturn::ERROR;
    }
    const int id = std::stoi(id_it->second);
    joint_ids_[i] = static_cast<uint8_t>(id);

    JointParams params(joint.parameters.begin(), joint.parameters.end());
    if (const auto yaml_it = yaml_by_id.find(id); yaml_it != yaml_by_id.end()) {
      params = feetech_ros2_driver::merge_joint_params(yaml_it->second, joint.parameters);
    }
    try {
      if (const auto it = params.find("center_tick"); it != params.end()) {
        joint_center_ticks_[i] = std::stoi(it->second);
      }
      if (const auto it = params.find("direction"); it != params.end()) {
        joint_directions_[i] = std::stoi(it->second);
      }
    } catch (const std::exception& error) {
      spdlog::error("Joint '{}': invalid coordinate mapping: {}", joint.name, error.what());
      return CallbackReturn::ERROR;
    }
    if (joint_center_ticks_[i] < 0 || joint_center_ticks_[i] > 4095 ||
        (joint_directions_[i] != -1 && joint_directions_[i] != 1)) {
      spdlog::error("Joint '{}': center_tick must be 0..4095 and direction must be -1 or 1", joint.name);
      return CallbackReturn::ERROR;
    }

    if (const auto result = communication_protocol_->disable_torque(joint_ids_[i]); !result) {
      spdlog::error("Joint '{}' disable torque -> {}", joint.name, result.error());
      return CallbackReturn::ERROR;
    }

    for (const auto& [parameter_name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                                  {"d_coefficient", SMS_STS_D_COEF},
                                                  {"i_coefficient", SMS_STS_I_COEF},
                                                  {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                                  {"return_delay_time", SMS_STS_RETURN_DELAY},
                                                  {"acceleration", SMS_STS_ACC}}) {
      if (const auto it = params.find(parameter_name); it != params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(it->second))));
        if (!result) {
          spdlog::error("Joint '{}' write {} -> {}", joint.name, parameter_name, result.error());
          return CallbackReturn::ERROR;
        }
      }
    }
    for (const auto& [parameter_name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                                  {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                                  {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                                  {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      if (const auto it = params.find(parameter_name); it != params.end()) {
        std::array<uint8_t, 2> buffer{};
        feetech_driver::to_sts(&buffer[0], &buffer[1], std::stoi(it->second));
        if (const auto result = communication_protocol_->write(joint_ids_[i], address, buffer); !result) {
          spdlog::error("Joint '{}' write {} -> {}", joint.name, parameter_name, result.error());
          return CallbackReturn::ERROR;
        }
      }
    }
    if (const auto it = params.find("homing_offset"); it != params.end()) {
      std::array<uint8_t, 2> buffer{};
      const int value = feetech_driver::encode_sign_magnitude(std::stoi(it->second), SMS_STS_SIGN_BIT_HOMING_OFFSET);
      feetech_driver::to_sts(&buffer[0], &buffer[1], value);
      if (const auto result = communication_protocol_->write(joint_ids_[i], SMS_STS_OFS_L, buffer); !result) {
        spdlog::error("Joint '{}' write homing_offset -> {}", joint.name, result.error());
        return CallbackReturn::ERROR;
      }
    }
    if (const auto result = communication_protocol_->lock_eprom(joint_ids_[i]); !result) {
      spdlog::error("Joint '{}' lock EEPROM -> {}", joint.name, result.error());
      return CallbackReturn::ERROR;
    }
    if (!joint.command_interfaces.empty()) {
      if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
        spdlog::error("Joint '{}' enable torque -> {}", joint.name, result.error());
        return CallbackReturn::ERROR;
      }
    }
  }

  const auto models = joint_ids_ | ranges::views::transform([&](const auto id) {
                        return communication_protocol_->read_model_number(id)
                            .and_then(feetech_driver::get_model_name)
                            .and_then(feetech_driver::get_model_series);
                      });
  if (std::ranges::any_of(models, [](const auto& series) {
        return !series.has_value() || series.value() != feetech_driver::ModelSeries::kSts;
      })) {
    spdlog::error("Only responding STS-series servos are supported");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SoArm101PgripperHardwareInterface::export_state_interfaces() {
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  state_hw_currents_.resize(info_.joints.size(), 0.0);
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
    interfaces.emplace_back(info_.joints[i].name, "current", &state_hw_currents_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SoArm101PgripperHardwareInterface::export_command_interfaces() {
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }
  return interfaces;
}

hardware_interface::return_type SoArm101PgripperHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
  std::vector<std::array<uint8_t, 15>> data;
  data.reserve(joint_ids_.size());
  if (const auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    spdlog::error("SO-ARM101 read -> {}", result.error());
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& item) {
    const auto& [index, readings] = item;
    state_hw_positions_[index] = ticks_to_radians(
        feetech_driver::from_sts({.low = readings[0], .high = readings[1]}), joint_directions_[index], joint_center_ticks_[index]);
    state_hw_velocities_[index] = joint_directions_[index] * feetech_driver::to_radians(
        feetech_driver::from_sts({.low = readings[2], .high = readings[3]}));
    state_hw_currents_[index] = feetech_driver::from_sts({.low = readings[13], .high = readings[14]});
  });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoArm101PgripperHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
  std::vector<uint8_t> ids;
  std::vector<int> positions;
  std::vector<int> speeds;
  std::vector<int> accelerations;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      ids.push_back(joint_ids_[i]);
      positions.push_back(radians_to_ticks(hw_positions_[i], joint_directions_[i], joint_center_ticks_[i]));
      speeds.push_back(2400);
      accelerations.push_back(50);
    }
  }
  if (!ids.empty()) {
    if (const auto result = communication_protocol_->sync_write_position(ids, positions, speeds, accelerations); !result) {
      spdlog::error("SO-ARM101 write -> {}", result.error());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

CallbackReturn SoArm101PgripperHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  hw_positions_ = state_hw_positions_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SoArm101PgripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
  const auto parameters = std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result = communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, parameters); !result) {
    spdlog::error("SO-ARM101 deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace so_arm101_pgripper_hardware

PLUGINLIB_EXPORT_CLASS(so_arm101_pgripper_hardware::SoArm101PgripperHardwareInterface,
                       hardware_interface::SystemInterface)

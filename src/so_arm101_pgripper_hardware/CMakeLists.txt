cmake_minimum_required(VERSION 3.16)
project(so_arm101_pgripper_hardware)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(feetech_driver CONFIG REQUIRED)
find_package(feetech_ros2_driver REQUIRED)
find_package(fmt REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(range-v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tl-expected QUIET)
if(NOT tl-expected_FOUND)
  find_package(tl_expected REQUIRED)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

add_library(${PROJECT_NAME} src/so_arm101_pgripper_hardware.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../feetech_ros2_driver/include
          ${CMAKE_CURRENT_SOURCE_DIR}/../feetech_ros2_driver/feetech_driver/include)
target_link_libraries(${PROJECT_NAME}
  PRIVATE feetech_driver::communication_protocol
          yaml-cpp
          fmt::fmt
          hardware_interface::hardware_interface
          pluginlib::pluginlib
          rclcpp::rclcpp
          rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface so_arm101_pgripper_hardware_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tick_conversion_test test/tick_conversion_test.cpp)
  target_include_directories(tick_conversion_test PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../feetech_ros2_driver/feetech_driver/include)
  target_link_libraries(tick_conversion_test fmt::fmt)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(feetech_ros2_driver hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()

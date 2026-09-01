cmake_minimum_required(VERSION 3.16)
project(so_arm101_pgripper_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/servo_keyboard_input.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_keyboard_input
)

install(PROGRAMS
  scripts/servo_joy_input.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_joy_input
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(moveit_config_test test/test_moveit_config.py TIMEOUT 60)
  ament_add_pytest_test(servo_config_test test/test_servo_config.py TIMEOUT 60)
  ament_add_pytest_test(servo_inputs_test test/test_servo_inputs.py TIMEOUT 60)
  ament_add_pytest_test(vendored_servo_patch_test test/test_vendored_servo_patch.py TIMEOUT 60)
endif()

ament_package()

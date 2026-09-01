cmake_minimum_required(VERSION 3.16)
project(so_arm101_pgripper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY LICENSES
  DESTINATION share/${PROJECT_NAME}
)
install(FILES ATTRIBUTION.md README.md
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/calibrate_so_arm101_pgripper.py
  scripts/calibration_common.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_test test/test_description.py TIMEOUT 60)
  ament_add_pytest_test(calibration_test test/test_calibration.py TIMEOUT 60)
  ament_add_pytest_test(real_launch_test test/test_real_launch.py TIMEOUT 60)
endif()

ament_package()

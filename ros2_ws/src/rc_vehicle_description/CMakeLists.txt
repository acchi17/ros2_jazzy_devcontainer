cmake_minimum_required(VERSION 3.8)
project(rc_vehicle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The package has no source files, so skip checks that only apply to code
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

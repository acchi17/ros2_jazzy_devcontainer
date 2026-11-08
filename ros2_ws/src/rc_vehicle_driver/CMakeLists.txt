cmake_minimum_required(VERSION 3.8)
project(rc_vehicle_driver)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Real GPIO output needs liblgpio (apt: liblgpio-dev, Raspberry Pi image only).
# Without it the plugin still builds and works with use_mock_gpio:=true.
find_library(LGPIO_LIBRARY lgpio)
find_path(LGPIO_INCLUDE_DIR lgpio.h)

add_library(rc_vehicle_driver SHARED src/rc_vehicle_driver_hardware.cpp)
target_include_directories(rc_vehicle_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rc_vehicle_driver PUBLIC cxx_std_17)
ament_target_dependencies(rc_vehicle_driver PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
if(LGPIO_LIBRARY AND LGPIO_INCLUDE_DIR)
  target_compile_definitions(rc_vehicle_driver PRIVATE RC_VEHICLE_DRIVER_HAS_LGPIO)
  target_include_directories(rc_vehicle_driver PRIVATE ${LGPIO_INCLUDE_DIR})
  target_link_libraries(rc_vehicle_driver PRIVATE ${LGPIO_LIBRARY})
else()
  message(WARNING "lgpio not found: rc_vehicle_driver supports only use_mock_gpio:=true")
endif()

pluginlib_export_plugin_description_file(hardware_interface rc_vehicle_driver_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS rc_vehicle_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Project style linters are not used in this repository
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pwm_calibration test/test_pwm_calibration.cpp)
  target_include_directories(test_pwm_calibration PRIVATE include)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(rc_vehicle_driver)
ament_export_targets(export_${PROJECT_NAME})
ament_package()

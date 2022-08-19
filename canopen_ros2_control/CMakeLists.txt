cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/canopen_system.cpp
)

# prevent pluginlib from using boost
target_compile_definitions(canopen_ros2_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface canopen_ros2_control.xml)

ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
target_include_directories(test_canopen_system PRIVATE include)
ament_target_dependencies(
  test_canopen_system
  hardware_interface
  pluginlib
  ros2_control_test_assets
)

ament_auto_package(INSTALL_TO_SHARE config launch urdf)

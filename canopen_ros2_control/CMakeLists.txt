cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/canopen_system.cpp
  src/cia402_system.cpp
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface canopen_ros2_control.xml)

if(BUILD_TESTING)
  ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
  target_include_directories(test_canopen_system PRIVATE include)
  ament_target_dependencies(
    test_canopen_system
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config launch urdf)

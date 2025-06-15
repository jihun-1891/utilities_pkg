cmake_minimum_required(VERSION 3.8)
project(utilities_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(px4_msgs REQUIRED)

add_executable(keyboard_control_node src/keyboard_control_node.cpp)
ament_target_dependencies(keyboard_control_node rclcpp geometry_msgs px4_msgs tf2_ros tf2_geometry_msgs)
install(TARGETS keyboard_control_node DESTINATION lib/${PROJECT_NAME})

ament_package()

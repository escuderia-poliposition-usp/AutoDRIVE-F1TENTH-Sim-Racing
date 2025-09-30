cmake_minimum_required(VERSION 3.8)
project(f1tenth_dynsim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(f1tenth_interface_plugin SHARED
  src/f1tenth_interface_plugin.cpp
)

ament_target_dependencies(f1tenth_interface_plugin
  rclcpp
  gazebo_ros
  geometry_msgs
  std_msgs
)

target_link_libraries(f1tenth_interface_plugin
  ${gazebo_ros_LIBRARIES}
)

install(TARGETS f1tenth_interface_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS
  scripts/twist_to_actuators.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds models config 
  DESTINATION share/${PROJECT_NAME})

ament_package()

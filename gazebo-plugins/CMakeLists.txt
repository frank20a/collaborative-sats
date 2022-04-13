cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ROS
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Chaser Controller
add_library(chaser_controller SHARED chaser_controller.cc)
target_link_libraries(chaser_controller ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})

# Test Plugin
add_library(test_plugin SHARED test_plugin.cc)
target_link_libraries(test_plugin ${GAZEBO_LIBRARIES})
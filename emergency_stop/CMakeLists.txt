cmake_minimum_required(VERSION 3.0.2)
project(emergency_stop)

## Add support for C++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES
  EmergencyBreak.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)




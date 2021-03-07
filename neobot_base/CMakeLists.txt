cmake_minimum_required(VERSION 3.0.2)
project(neobot_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2
  nav_msgs
  neo_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES neobot_base
 CATKIN_DEPENDS roscpp rospy tf2_ros tf2 nav_msgs neo_msgs std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(neobot_base
#   src/neobot_base.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(neobot_base src/neobot_base.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(neobot_base neo_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(neobot_base
  ${catkin_LIBRARIES}
)

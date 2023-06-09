cmake_minimum_required(VERSION 3.0.2)
project(basic_planning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs visualization_msgs tf2 tf2_ros)

## Declare a catkin package
catkin_package()

# The rsmotion library requires the "-fpermissive" flag.
# TODO: fix this.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(
  rsmotion

  src/rsmotion/rsmotion.cpp
)

add_executable(
  basic_planning_main
  
  src/basic_planning_main.cpp
)

target_link_libraries(
  basic_planning_main
  rsmotion
  ${catkin_LIBRARIES}
)

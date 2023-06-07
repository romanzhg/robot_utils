cmake_minimum_required(VERSION 3.0.2)
project(basic_planning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs visualization_msgs tf2 tf2_ros)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
  basic_planning_main
  
  src/basic_planning_main.cpp
)

target_link_libraries(
  basic_planning_main
  ${catkin_LIBRARIES}
)

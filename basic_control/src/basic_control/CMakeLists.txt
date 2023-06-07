cmake_minimum_required(VERSION 3.0.2)
project(basic_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs visualization_msgs tf2 tf2_ros)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

add_library(basic_mpc src/basic_control/mpc.cpp)

target_include_directories(
  basic_mpc
  PUBLIC /usr/local/include
)

# Note if the program is linked with mpc.h, then needs to run
#   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# to add /usr/local/lib to the loader path.
target_link_libraries(
  basic_mpc
  ipopt
)

add_executable(
  basic_control_main
  
  src/basic_control_main.cpp
)

target_link_libraries(
  basic_control_main
  basic_mpc
  ${catkin_LIBRARIES}
)

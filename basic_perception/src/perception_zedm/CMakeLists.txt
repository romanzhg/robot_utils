cmake_minimum_required(VERSION 3.0.2)
project(basic_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs visualization_msgs tf2 tf2_ros)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

add_executable(
  basic_perception_main
  
  src/basic_perception_main.cpp
)

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

target_link_libraries(
  basic_perception_main
  ${catkin_LIBRARIES}

  ${ZED_LIBS}
)

add_executable(
  zed_record
  
  src/zed_record.cpp
)

target_link_libraries(
  zed_record

  ${ZED_LIBS}
)
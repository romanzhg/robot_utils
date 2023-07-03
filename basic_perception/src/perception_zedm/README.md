# README
To run:
Note this is a ros/catkin workspace, requires ros noetic and stereolabs zed driver 3.8.

Directory structure:

ws_dir/src/basic_perception

corresponds to:

**workspace directory**/src/**package name**

To compile and run:

in **workspace directory**:

catkin_make -DCMAKE_BUILD_TYPE=Debug

catkin_make -DCMAKE_BUILD_TYPE=Release
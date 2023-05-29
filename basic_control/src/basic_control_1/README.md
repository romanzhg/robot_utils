# README
To run:
Note this is a ros/catkin workspace, requires ros noetic.

Directory structure:
ws_dir/src/basic_control_1
corresponds to:
<workspace directory>/src/<package name>

To compile and run:
in <workspace directory>:
catkin_make -DCMAKE_BUILD_TYPE=Debug
catkin_make -DCMAKE_BUILD_TYPE=Release

then run the output binary.
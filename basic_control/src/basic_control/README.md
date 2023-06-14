# README
To run:
Note this is a ros/catkin workspace, requires ros noetic.

Directory structure:

ws_dir/src/basic_control

corresponds to:

**workspace directory**/src/**package name**

To compile and run:

in **workspace directory**:

catkin_make -DCMAKE_BUILD_TYPE=Debug

catkin_make -DCMAKE_BUILD_TYPE=Release

then run the output binary. If the binary is compiled with mpc, remember to run 

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# MAPF

Multi-agent path finding implementations.
最优多机器人寻路算法实现。

## Background

Test data from https://movingai.com/benchmarks/mapf/index.html.

## Notes

### To Run

- Configure the solvers to run by change ./src/flags.cpp.

- Enter root directory, make directory build, enter build, run `cmake .. && make && ./bin/local_test_main`.
Output will be written to ./output.

- To print the result, enter the build directory,
run `python3 ../src/utilities/mapf_gui.py <path to the output file>`.

### General

- Code in solvers/cbs duplicates with code solvers/icbs, this is intentional, results by the two versions
of cbs should be checked against each other.

### Notes

- Use shortest path distance instead of manhattan distance speeds up significantly.
- The basic astar can finish in 30s with up to 8 robots in maze maze-32-32-2-even-1.scen.
# robot_utils

这个repo包含了一些机器人/自动化方面可以用到的软件工程模块和算法模块。

## basic_control
一些基本的控制算法，实现自动驾驶中的横向控制。这个文件夹是一个ros workspace，代码开发于ros noetic/ubuntu 20.04。

包含
- Cross track error PID
- Pure pursuit
- LQR加前馈
- Model based search (搜索所有可能的动作序列，根据cost最低的行动)

### todo
参考线应转换到车体坐标系下，然后用一三次曲线去fit，结果用来求cross track error。

## rrt_planner_py
RRT规划算法实现，代码框架取自waterloo cs477的作业。
### todo
用kd-tree来加速最近点搜索。

## basic_lam
一个mapper的实现，根据bag中激光雷达数据制图。代码框架取自waterloo cs477的作业。

一个localizer的实现，优化基于最小二乘法。代码框架取自waterloo cs477的作业。

## monte_carlo_loc
一个基本的particle filter定位程序，播bag验证。代码框架取自waterloo cs477的作业。

## kiva_simulation
仓储物流环境下的多机器人持续规划算法实现和模拟。

## mapf
一些最优多机器人寻路算法的实现。实现算法包括astar, cbs, icbs。
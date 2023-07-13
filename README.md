# robot_utils

这个repo包含了一些机器人/自动化方面可以用到的软件工程模块和算法模块。

## basic_control
一些基本的控制算法，实现自动驾驶中的横向控制。这个文件夹是一个ros workspace，代码开发于ros noetic/ubuntu 20.04。

包含
- Cross track error PID
- Pure pursuit
- LQR加前馈
- Model based search (根据运动模型预测机器人状态，搜索所有可能的动作序列，取cost最低的行动)
- Model predictive control (根据运动模型预测机器人状态，使用ipopt做非线性优化，取cost最低的行动)

## basic_planning
针对自动驾驶中的泊车路径规划场景，实现了一个简化了的简化hyper Astar + Reeds Shepp。这个文件夹是一个ros workspace，代码开发于ros noetic/ubuntu 20.04。

Reeds Shepp库取自https://github.com/BasGeertsema/rsmotion。

## basic_perception
实现点云数据中的平面提取和显示，点云数据由zed mini双目相机输出（运行程序需要安装zed sdk 3.8及Eigen）。

## rrt_planner_py
RRT规划算法实现，代码框架取自waterloo cs477的作业。

## basic_lam
一个mapper的实现，根据bag中激光雷达数据制图。代码框架取自waterloo cs477的作业。

一个localizer的实现，优化基于最小二乘法。代码框架取自waterloo cs477的作业。

一个思岚激光雷达手持建图的所用的cartographer 2d参数配置。

## monte_carlo_loc
一个基本的particle filter定位程序，播bag验证。代码框架取自waterloo cs477的作业。

## kiva_simulation
仓储物流环境下的多机器人持续规划算法实现和模拟。

## mapf
一些最优多机器人寻路算法的实现。实现算法包括astar, cbs, icbs。
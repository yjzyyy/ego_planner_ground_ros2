# EGO Planner Ground ROS 2

## 项目简介

本项目基于 [原 ROS 版本项目名称] 修改适配为 ROS 2 版本，已在 Ubuntu 22.04 系统上完成测试。

## 注意事项

**重要提示**：直接全局编译可能出现卡死问题，请按照以下编译顺序操作。

## 安装与编译

### 1. 单独编译 plan_env 包
```bash
colcon build --packages-select plan_env
```
2. 整体项目编译
```bash
colcon build
```
3. 加载环境变量
```bash

source install/setup.bash
```
运行说明
第一步：启动车辆仿真器

在第一个终端窗口中执行：
```bash

ros2 launch vehicle_simulator system_indoor.launch
```
第二步：启动 EGO Planner

打开新的终端窗口，执行：
```bash

# 加载环境变量
source install/setup.bash
```
# 启动规划器
``
ros2 launch ego_planner run_in_sim.launch.py
```
系统要求

    Ubuntu 22.04

    ROS 2 (具体版本请根据实际情况填写)

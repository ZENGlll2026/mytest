
# 基于 ROS 2 的turtble导航画RM
## 1.项目介绍

基于 ROS2 Humble 开发的 turtlesim 小乌龟自动导航节点，实现小乌龟依次遍历 8 个预设坐标点，到达终点后停止运动。


各功能包如下：
- test_turtle 


## 2.使用方法

本项目开发平台信息如下：

- 系统版本： Ubunt22.04
- ROS 版本：ROS 2 Humble

### 2.1安装

本项目依赖 ROS2 核心库与 turtlesim 仿真工具，执行以下命令安装：
```
sudo apt install ros-humble-turtlesim ros-humble-rclcpp ros-humble-geometry-msgs
```


### 2.2运行

安装完成依赖后，可以使用 colcon 工具进行构建和运行。

构建功能包

```
colcon build
```

运行海龟模拟器节点

```
ros2 run turtlesim turtlesim_node
```
运行控制节点

```
source install/setup.bash
ros2 run test_turtle turtle_node_RM
```




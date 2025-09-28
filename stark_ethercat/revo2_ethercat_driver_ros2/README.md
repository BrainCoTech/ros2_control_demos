# Revo2 EtherCAT ROS2 驱动包

## 概述

本包为 BrainCo Revo2 灵巧手提供基于 EtherCAT 的 ROS2 控制接口，支持 6 个手指关节的实时控制与状态反馈。通过 ros2_control 框架实现硬件抽象。

## 快速开始

### 系统环境

- Ubuntu 22.04
- ROS2 Humble
- EtherCAT 主站 (IgH EtherCAT Master)
- Python 3.8+

### 安装依赖

```bash
# 安装 Python 依赖
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs

# 确保 EtherCAT 主站已安装并运行
sudo systemctl status ethercat
```

### 编译

```bash
# 进入工作空间根目录
cd <your_workspace>

# 编译相关包
colcon build --symlink-install --packages-select stark_ethercat_interface stark_ethercat_driver revo2_ethercat_driver_ros2

# 设置环境
source install/setup.bash
```

### 硬件连接检查

```bash
# 检查网络接口
ip link show

# 检查 EtherCAT 主站状态
sudo systemctl status ethercat

# 检查 EtherCAT 设备连接
sudo ethercat slaves

# 检查设备 SDO/PDO 信息
sudo ethercat sdos
sudo ethercat pdos
```

### 启动系统

```bash
# 启动 Revo2 系统
ros2 launch revo2_ethercat_driver_ros2 revo2_system.launch.py

# 调试模式（过滤域信息）
ros2 launch revo2_ethercat_driver_ros2 revo2_system.launch.py 2>&1 | grep -v "\[ros2_control_node-1\] Domain"
```

## 使用指南

### 基本控制

#### 1. 轨迹控制

```bash
# 发送轨迹命令
ros2 topic pub --once /revo2_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [thumb_flex_joint, thumb_abduct_joint, index_joint, middle_joint, ring_joint, pinky_joint], points: [{positions: [0.5,0.5,0.5,0.5,0.5,0.5], time_from_start: {sec: 2}}]}'

# 归零
ros2 topic pub --once /revo2_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [thumb_flex_joint, thumb_abduct_joint, index_joint, middle_joint, ring_joint, pinky_joint], points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 1}}]}'
```

#### 2. Action轨迹控制

```bash
# 单指依次弯曲
ros2 action send_goal /revo2_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [thumb_flex_joint, thumb_abduct_joint, index_joint, middle_joint, ring_joint, pinky_joint], points: [{positions: [0.5,0,0,0,0,0], time_from_start: {sec: 1}}, {positions: [0.5,0,1.4,0,0,0], time_from_start: {sec: 2}}, {positions: [0.5,0,1.4,1.4,0,0], time_from_start: {sec: 3}}, {positions: [0.5,0,1.4,1.4,1.4,0], time_from_start: {sec: 4}}, {positions: [0.5,0,1.4,1.4,1.4,1.4], time_from_start: {sec: 5}}]}}"
```

#### 3. 位置控制

```bash
# 启动：默认 launch 后仅加载控制器但未激活。按需切换：
# 激活位置控制器
ros2 run controller_manager spawner revo2_position_controller -c /controller_manager
# 控制器切换（如需停用轨迹控制器）
ros2 control switch_controllers --deactivate revo2_joint_trajectory_controller --activate revo2_position_controller

# 发送位置命令
ros2 topic pub --once /revo2_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"
ros2 topic pub --once /revo2_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0.8]}"
```

### 测试脚本

```bash
# 进入脚本目录
cd <package_path>/scripts/

# 交互式菜单
python3 test_revo2_ethercat.py --menu

# 直接测试
python3 test_revo2_ethercat.py --test all_fingers --amplitude 0.8
python3 test_revo2_ethercat.py --test individual
python3 test_revo2_ethercat.py --test home
```

### 状态监控

```bash
# 检查控制器状态
ros2 control list_controllers

# 监控关节状态
ros2 topic echo /joint_states

# 检查话题列表
ros2 topic list

# 检查硬件接口
ros2 control list_hardware_interfaces

# 检查可用的action接口
ros2 action list

# 监控action状态
ros2 action info /revo2_joint_trajectory_controller/follow_joint_trajectory

# 实时监控action反馈
ros2 action send_goal --feedback /revo2_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [thumb_flex_joint, thumb_abduct_joint, index_joint, middle_joint, ring_joint, pinky_joint], points: [{positions: [0.5,0.5,0.5,0.5,0.5,0.5], time_from_start: {sec: 3}}]}}"

```

## 关节映射

| 关节名称            | 描述       | 角度范围（弧度）      | 最大角度（度） |
|-------------------|------------|----------------------|--------------|
| thumb_flex_joint  | 拇指弯曲   | 0 ~ 1.03             | 59           |
| thumb_abduct_joint| 拇指外展   | 0 ~ 1.57             | 90           |
| index_joint       | 食指       | 0 ~ 1.41             | 81           |
| middle_joint      | 中指       | 0 ~ 1.41             | 81           |
| ring_joint        | 无名指     | 0 ~ 1.41             | 81           |
| pinky_joint       | 小指       | 0 ~ 1.41             | 81           |

## 联系方式

如有问题或建议，请联系开发团队。
# ros2_control_stark

```shell
colcon build --symlink-install
colcon build --symlink-install --packages-select ros2_control_stark

source install/setup.bash # bash

ros2 launch ros2_control_stark stark.launch.py
ros2 launch ros2_control_stark test_joint_trajectory_controller.launch.py

ros2 control list_controllers
ros2 topic list
ros2 topic echo /joint_states

ros2 topic info /forward_position_controller/commands
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.6, 0.6, 1.0, 1.0, 1.0, 1.0]}"
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.6, 0.6, 0, 0, 0, 0]}"

ros2 topic info /joint_trajectory_position_controller/joint_trajectory
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: ['thumb','thumb_aux','index','middle','ring','pinky'],
    points: [
      {positions: [0.6,0.6,1.0,1.0,1.0,1.0], time_from_start: {sec: 1}}
    ]
  }"
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: ['thumb','thumb_aux','index','middle','ring','pinky'],
    points: [
      {positions: [0.6,0.6,0,0,0,0], time_from_start: {sec: 1}}
    ]
  }"  
```

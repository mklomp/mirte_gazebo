# Installation

```sh
# In your ROS2 workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

# MIRTE master examples

```sh
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.py
```

Driving around:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Gripper open:
```sh
ros2 topic pub --once /mirte_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Gripper_joint'], points: [{positions: [0.25], time_from_start:{ sec: 1, nanosec: 0}}]}"
```

Gripper close:
```sh
ros2 topic pub --once /mirte_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Gripper_joint'], points: [{positions: [0.0], time_from_start:{ sec: 1, nanosec: 0}}]}"
```

Arm up:
```sh
ros2 topic pub --once /mirte_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['arm_Rot_joint', 'arm_Shoulder_joint', 'arm_Elbow_joint', 'arm_Wrist_joint'], points: [{positions: [0.0, 0.0, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```


Arm down:
```sh
ros2 topic pub --once /mirte_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['arm_Rot_joint', 'arm_Shoulder_joint', 'arm_Elbow_joint', 'arm_Wrist_joint'], points: [{positions: [0.0, -1.56, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```

# Installation

```sh
# In your ROS2 workspace
vcs import src/ < src/mirte-gazebo/sources.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

# MIRTE master examples

```sh
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.py
```

The following sequence of commands will let you pick up a cylinder, drive it somewhere else, and place it.

Spawn a cylinder:
```sh
ros2 run gazebo_ros spawn_entity.py -file $(pwd)/src/mirte-gazebo/urdf/cylinder.sdf -entity cylinder -x 1.39 -y .51
```

Gripper close:
```sh
ros2 topic pub --once /mirte_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Gripper_joint'], points: [{positions: [0.1], time_from_start:{ sec: 1, nanosec: 0}}]}"
```

Arm up:
```sh
ros2 topic pub --once /mirte_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['arm_Rot_joint', 'arm_Shoulder_joint', 'arm_Elbow_joint', 'arm_Wrist_joint'], points: [{positions: [0.0, 0.0, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```

Driving around:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Arm down:
```sh
ros2 topic pub --once /mirte_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['arm_Rot_joint', 'arm_Shoulder_joint', 'arm_Elbow_joint', 'arm_Wrist_joint'], points: [{positions: [0.0, -1.56, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```

Gripper open:
```sh
ros2 topic pub --once /mirte_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Gripper_joint'], points: [{positions: [-0.1], time_from_start:{ sec: 1, nanosec: 0}}]}"
```





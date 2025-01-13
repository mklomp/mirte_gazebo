## dependencies:
use rosdep.
## running MIRTE master:

```sh
roslaunch mirte_gazebo gazebo_mirte_master_empty.launch.py
```

Driving around:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Closing/opening the gripper:
```sh
ros2 topic pub --once /mirte_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Gripper_joint'], points: [{positions: [0.5], time_from_start:{ sec: 1, nanosec: 0}}]}"
```

And change the position value.


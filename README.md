## dependencies:

$ sudo apt install ros-humble-gazebo-ros ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard ros-humble-rviz2

not sure about htese dependencies:

$ sudo apt install ros-humble-controller-interface ros-humble-controller-manager ros-humble-controller-manager-msgs

## running:
WARNING: sometimes the teleop does not work. not sure yet why: rqt_graph looks ok

term1$ ros2 launch mirte_gazebo gazebo_mirte_diff_drive.launch.py

term2$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped

term3$ ros2 run rviz2 rviz2 -d mirte_gazebo.rviz


## TODO:
fix issue with teleop randomly not working
add ultrasonic sensors
make stl more realistic
make environement more realistic

## dependencies:

Have mirte-control and mirte-teleopkey installed

apt install ros-noetic-ros-control ros-noetic-ros-controllers
## running:

term1$ roslaunch mirte_gazebo gazebo_empty_world.launch
term2$ roslaunch mirte_gazebo spawn_duckietown.launch
term3$ roslaunch mirte_gazebo spawn_mirte.launch
term4$ rosrun rviz rviz -d mirte_gazebo.rviz

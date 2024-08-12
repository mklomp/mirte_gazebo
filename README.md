## dependencies:

Have mirte-control and mirte-teleopkey installed

apt install ros-noetic-ros-control ros-noetic-ros-controllers

Requires https://github.com/SyrianSpock/realsense_gazebo_plugin to be downloaded and built if you want an actual realsense. realsense-depth uses a simple depth-camera without the realsense extras.


## running:

```sh
roslaunch mirte_gazebo gazebo_all.launch
```

Controlling the Mirte:
```sh
roslaunch mirte_teleop teleopkey.launch
```

Change world by changing the `duckietown` argument in `launch/gazebo_duckietown_world.launch` to any map in the maps folder.

The map file is converted to `urdf/generated.world` by `scripts/buildMap.py` and converted to `worlds/generated.world` by xacro before launching Gazebo.
<!-- term1$ roslaunch mirte_gazebo gazebo_empty_world.launch
term2$ roslaunch mirte_gazebo spawn_duckietown.launch
term3$ roslaunch mirte_gazebo spawn_mirte.launch
term4$ rosrun rviz rviz -d mirte_gazebo.rviz -->

## Format
Format xml (.launch, .xacro and .world) by running 
```sh
./scripts/xmlformat.sh # uses xmllint
```
and
```sh
./scripts/pythonformat.sh # uses black
```

## VMWARE issues:
using vscode ssh: 
```sh
export DISPLAY=:0
```

# ROS2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source ./install/setup.zsh && eeee && ros2 launch mirte_gazebo rsp_lab4_generated.launch.xml --debug

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte/cmd_vel

ros2 run web_video_server web_video_server

## Reset position
lab3:
ign service -s /world/default/set_pose  --reqtype ignition.msgs.Pose   --reptype ignition.msgs.Boolean  --req 'name: "Mirte", position: {x: 1, y: 0.5, z: 0.1}' --timeout 3000


ros2 topic pub /mirte/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
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
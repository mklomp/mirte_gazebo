temr1$ gzserver -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so --verbose
term2$ gzclient --verbose


to get urf:
term3$ ros2 run xacro xacro  -o mirte.urdf ./src/mirte_gazebo/urdf/mirte.xacro

launch robot:
term3$ ros2 run gazebo_ros spawn_entity.py -file mirte.urdf -entity robot





TODO:
make diffdrive work
make camera work
make disatcne sensor work
create 1 launchfile
fix package usage in xacro (use packeg instead of file link)
use stl instead of box	


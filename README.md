## dependencies:

Install https://github.com/ARCC-RACE/duckietown-gazebo
And run the gazebo main with duckietown

And have mirte-control and mirte-teleopkey installed

## running:
WARNING: sometimes the teleop does not work. not sure yet why: rqt_graph looks ok

term1$ roslaunch duckietown-gazebo main.launch

term2$ roslaunch mirte-gazebo spawn.launch x:=2 y:=2

term3$ rorlaunch mirte-teleopkey teleopkey.launch

term4$ rosrun rviz rviz -d mirte_gazebo.rviz


## TODO:
add ultrasonic sensors
make stl more realistic
make environement more realistic

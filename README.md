term1$ ros2 launch mirte_gazebo gazebo_mirte_diff_drive.launch

term2$ os2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped


TODO:
make camera work
make disatcne sensor work
use stl instead of box	
cleanup setup.py with cuplicates

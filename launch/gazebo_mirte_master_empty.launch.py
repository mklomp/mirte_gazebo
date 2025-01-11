# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



import xacro


def generate_launch_description():

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    world_file_name = 'empty.world'
    world_path = os.path.join(pkg_gazebo_ros, 'worlds', world_file_name)

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
              launch_arguments={
                'world': os.path.join(os.path.join(get_package_share_directory('gazebo_ros')), 'empty.world'),
                'extra_gazebo_args': '-s libgazebo_ros_camera.so -s libgazebo_ros_ray_sensor.so -s libgazebo_ros_range.so --verbose'
              }.items()
        )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('mirte_master_description'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'mirte_master.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-Y', '0.0',
                                   '-entity', 'mirte_master'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
#            "pid_wheels_controller",
            "mirte_base_controller",
        ],
    )

    # Due to the lack of a timeout in the gazebo planar move plugin
    # we need to set the timeout through the twist_mux
    zero_cmd_vel_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/zero_cmd_vel', 'geometry_msgs/msg/Twist',
             '{}', '-r', '100'],
    )

    config = os.path.join(
        get_package_share_directory('mirte_gazebo'),
        'config',
        'twist_mux.yaml'
        )

    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', '/cmd_vel')},
            parameters=[
                {'use_sim_time': True},
                config]
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
           )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        twist_mux,
        zero_cmd_vel_publisher
    ])

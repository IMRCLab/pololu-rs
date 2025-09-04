import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('crazyflie'), 'launch'
                        ),
                        '/launch.py',
                    ]
                ),
                launch_arguments={
                    'backend': 'cflib',
                    'gui': 'false',
                    'teleop': 'false',
                    'mocap': 'false',
                }.items(),
            ),
            Node(
                package='crazyflie',
                executable='vel_mux.py',
                name='vel_mux',
                output='screen',
                parameters=[
                    {'hover_height': 0.3},
                    {'incoming_twist_topic': '/cmd_vel'},
                    {'robot_prefix': '/cf231'},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch/online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/slam_params.yaml',
                    ),
                    'use_sim_time': 'False',
                }.items(),
            ),
        ]
    )

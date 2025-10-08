import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tracking_params = os.path.join(
        get_package_share_directory('pololu'),
        'config',
        'motion_capture.yaml')

    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[tracking_params]
        ),
    ])

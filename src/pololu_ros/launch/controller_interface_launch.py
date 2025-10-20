import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Controller interface params
    controller_params = os.path.join(
        get_package_share_directory('pololu'),
        'config',
        'controller_interface.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=controller_params,
        description='Path to controller interface config file'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    return LaunchDescription([
        config_arg,
        device_arg,
        
        # Single joy node for controller interface
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            remappings=[('joy', 'joy1')],
            # parameters=[{'device_name':'C1'}]
            parameters=[{'dev':'/dev/input/js0'}], # use "ros2 run joy joy_enumerate_devices" to find mapping
            output='screen'
        ),
        
        # Controller interface node
        Node(
            package='pololu',
            executable='controller_interface',
            name='controller_interface',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
            emulate_tty=True,  # For colored output
        ),
    ])

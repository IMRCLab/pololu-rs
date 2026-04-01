import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # teleop params
    teleop_params = os.path.join(
        get_package_share_directory('pololu'),
        'config',
        'teleop.yaml')

    return LaunchDescription([
        Node(
            package='pololu',
            executable='teleop',
            name='teleop',
            parameters=[teleop_params],
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            remappings=[('joy', 'joy1')],
            # parameters=[{'device_name':'C1'}]
            parameters=[{'dev':'/dev/input/js0'}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            remappings=[('joy', 'joy2')],
            # parameters=[{'device_name':'C1'}]
            parameters=[{'dev':'/dev/input/js1'}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            remappings=[('joy', 'joy3')],
            # parameters=[{'device_name':'C1'}]
            parameters=[{'dev':'/dev/input/js3'}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            remappings=[('joy', 'joy4')],
            # parameters=[{'device_name':'C1'}]
            parameters=[{'dev':'/dev/input/js4'}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        ),
        # Node(
        #     package='joy_linux',
        #     executable='joy_linux_node',
        #     remappings=[('joy', 'joy2')],
        #     # parameters=[{'device_name':'C1'}]
        #     parameters=[{'device_id':2}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        # ),
        # Node(
        #     package='joy_linux',
        #     executable='joy_linux_node',
        #     remappings=[('joy', 'joy3')],
        #     # parameters=[{'device_name':'C1'}]
        #     parameters=[{'device_id':3}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        # ),
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node4',
        #     remappings=[('joy', 'joy4')],
        #     # parameters=[{'device_name':'C1'}]
        #     parameters=[{'device_id':4}] # use "ros2 run joy joy_enumerate_devices" to find mapping
        # ),
    ])

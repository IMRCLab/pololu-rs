from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='Pololu09',
            description='Name of the robot in motion capture system'
        ),
        DeclareLaunchArgument(
            #get poses from mocap like in controller interface for pololu
            'mocap_topic',
            default_value='/poses',
            description='Topic for motion capture poses'
        ),
        DeclareLaunchArgument(
            #topic for publishing commands from controller: linear and angular velocity
            'cmd_unicycle_topic',
            default_value='/cmd_unicycle',
            description='Topic to publish unicycle actions (linear and angular velocity)'
        ),
        DeclareLaunchArgument(
            #control loop execution frequency
            'control_rate',
            default_value='10', #0.1s in controller simulation code
            description='Control loop rate in Hz'
        ),
        
        Node(
            package='wmr_controller',
            executable='wmr_controller_node',
            name='wmr_controller',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'mocap_topic': LaunchConfiguration('mocap_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_unicycle_topic'),
                'control_rate': LaunchConfiguration('control_rate'),
            }]
        ),
    ])

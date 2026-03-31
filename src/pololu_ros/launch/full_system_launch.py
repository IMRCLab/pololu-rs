import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────────────
    pololu_share = get_package_share_directory('pololu')
    controller_params = os.path.join(pololu_share, 'config', 'controller_interface.yaml')

    # Derive the colcon workspace root from the install path:
    #   .../install/pololu/share/pololu  →  go up 4 levels  →  workspace root
    ws_root = os.path.normpath(os.path.join(pololu_share, '..', '..', '..', '..'))
    mqtt_tracking_dir = os.path.join(ws_root, 'mqtt-tracking-system')

    # ── Launch arguments ──────────────────────────────────────────
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=controller_params,
        description='Path to controller interface config file')

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path')

    # ── MQTT tracking system (standalone scripts) ─────────────────
    aruco_server = ExecuteProcess(
        cmd=['python3', 'server.py'],
        cwd=mqtt_tracking_dir,
        name='aruco_server',
        output='screen',
    )

    mqtt_bridge = ExecuteProcess(
        cmd=['python3', 'mqtt_to_ros_bridge.py'],
        cwd=mqtt_tracking_dir,
        name='mqtt_to_ros_bridge',
        output='screen',
    )

    # ── ROS 2 nodes ──────────────────────────────────────────────
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        remappings=[('joy', 'joy1')],
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen',
    )

    controller_interface = Node(
        package='pololu',
        executable='controller_interface',
        name='controller_interface',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        device_arg,
        aruco_server,
        mqtt_bridge,
        joy_node,
        controller_interface,
    ])

#!/usr/bin/env python3
"""
MQTT -> ROS 2 Bridge Node

Subscribes to the MQTT topic (configured in config.yaml) and republishes
all detected robots as a motion_capture_tracking_interfaces/msg/NamedPoseArray
on topic /poses — matching the motion_capture_tracking format used by mocap subscribers controller_interface, mocap_broadcast and ann-cmgs controller

Each robot is identified by its name (e.g. "Pololu10") via the 'robots'
mapping in config.yaml (ArUco marker ID -> robot name end digits).

The 2D position [x, y] is mapped to pose.position.x/.y (z = 0).
The yaw angle (degrees) is converted to a quaternion around the z axis.

Usage:
    Start server.py from the mqtt-tracking-system repo root first, then run this bridge node:
    python3 mqtt_to_ros_bridge.py
    python3 mqtt_to_ros_bridge.py path/to/config.yaml

Requires:
    - webcam: BRIO 4K
    - source ROS2 
    - Source your ROS2 workspace that contains motion_capture_tracking_interfaces
    - e.g.: source install/setup.bash in that workspace

"""
import json
import math
import sys
import os

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
import yaml


DEFAULT_CONFIG_PATH = "config.yaml"


def load_mqtt_config(config_path: str) -> dict:
    """Load the mqtt and robots sections from config.yaml."""
    if not os.path.exists(config_path):
        print(f"ERROR: Config file '{config_path}' not found.")
        sys.exit(1)
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if "mqtt" not in cfg:
        print("ERROR: config.yaml is missing the 'mqtt' section.")
        sys.exit(1)

    # Build marker_id (int) -> robot_name (str) mapping
    robot_map = {}
    for marker_id, name in (cfg.get("robots") or {}).items():
        robot_map[str(marker_id)] = str(name)

    return cfg["mqtt"], robot_map

# system is only for 2D applications ... TODO: solution for future 2.5D experiments
def yaw_to_quaternion(yaw_deg: float):
    """Convert a yaw angle in degrees to a (x, y, z, w) quaternion (rotation around Z)."""
    yaw = math.radians(yaw_deg)
    return (
        0.0,                    # qx
        0.0,                    # qy
        math.sin(yaw / 2.0),   # qz
        math.cos(yaw / 2.0),   # qw
    )


class MqttToRosBridge(Node):
    def __init__(self, mqtt_cfg: dict, robot_map: dict):
        super().__init__("mqtt_to_ros_bridge")

        self.broker = str(mqtt_cfg["broker"])
        self.port = int(mqtt_cfg["port"])
        self.topic = str(mqtt_cfg["topic"])
        self.robot_map = robot_map  # marker_id (str) -> robot_name (str)

        # Match QoS used by mocap subscribers (BEST_EFFORT)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Single publisher on /poses for all robots (NamedPoseArray)
        self._publisher = self.create_publisher(NamedPoseArray, "/poses", qos)

        if self.robot_map:
            self.get_logger().info(f"Robot name mapping: {self.robot_map}")
        else:
            self.get_logger().warn("No 'robots' mapping in config — will use marker IDs as names")

        # --- MQTT client setup ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message

        self.get_logger().info(
            f"Connecting to MQTT broker {self.broker}:{self.port} "
            f"and subscribing to '{self.topic}' ..."
        )
        self.mqtt_client.connect(self.broker, self.port, keepalive=60)

        # Non-blocking: processes MQTT traffic in a background thread
        self.mqtt_client.loop_start()

        # Timer to spin ROS callbacks (not strictly needed but keeps the node alive)
        self.create_timer(0.01, lambda: None)

    # ---- MQTT callbacks -------------------------------------------------- #

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker. Subscribing to '{self.topic}'")
            client.subscribe(self.topic)
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def _on_message(self, client, userdata, msg):
        """Called whenever a message arrives on the subscribed MQTT topic."""
        try:
            data: dict = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f"Failed to decode MQTT message: {e}")
            return

        stamp = self.get_clock().now().to_msg()

        # Build a single NamedPoseArray containing all robots
        array_msg = NamedPoseArray()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = "world"

        for marker_id, robot_state in data.items():
            position = robot_state.get("position", [0.0, 0.0])
            angle = robot_state.get("angle", 0.0)

            # Resolve robot name from config, fall back to marker ID
            robot_name = self.robot_map.get(marker_id, marker_id)

            # Build NamedPose
            named_pose = NamedPose()
            named_pose.name = robot_name

            named_pose.pose.position.x = float(position[0])
            named_pose.pose.position.y = float(position[1])
            named_pose.pose.position.z = 0.0

            qx, qy, qz, qw = yaw_to_quaternion(angle)
            named_pose.pose.orientation.x = qx
            named_pose.pose.orientation.y = qy
            named_pose.pose.orientation.z = qz
            named_pose.pose.orientation.w = qw

            array_msg.poses.append(named_pose)

        self._publisher.publish(array_msg)

    # ---- Cleanup --------------------------------------------------------- #

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    config_path = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_CONFIG_PATH
    mqtt_cfg, robot_map = load_mqtt_config(config_path)

    rclpy.init(args=args)
    node = MqttToRosBridge(mqtt_cfg, robot_map)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Vector3
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np


class ReferenceTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('reference_trajectory_publisher')
        self.get_logger().info('=== Node init called ===')
        
        # Use BEST_EFFORT QoS with depth 1 to match subscriber and avoid buffering
        cmd_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Mocap QoS (typically BEST_EFFORT with depth 10 for pose data)
        mocap_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for control actions 
        self.cmd_vel_pub = self.create_publisher(Vector3, '/cmd_unicycle', cmd_qos_profile)
        self.get_logger().info('=== Publisher created ===')
        
        # Subscribe to mocap poses
        self.mocap_subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            mocap_qos_profile
        )
        self.get_logger().info('=== Mocap subscription created ===')
        
        # Store latest poses
        self.latest_poses = {}  # Dict mapping robot name to pose
        
        # Timer for publishing references ... control frequency 10 Hz
        self.timer = self.create_timer(0.1, self.publish_reference)
        self.get_logger().info('=== Timer created ===')
        
        self.t = self.get_clock().now() #ros2 time
        self.get_logger().info('Reference trajectory publisher started ...') #last seen print
    
    def publish_reference(self):
        """Publish a circular reference trajectory"""
        #self.get_logger().info('=== TIMER CALLBACK FIRED ===')
        #print("Publishing reference trajectory")
        #simple circle trajectory for testing
        radius = 0.3 #30 cm radius
        omega = 0.71  # rad/s (quarter circle in one second)

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.t).nanoseconds / 1e9  #elapsed time in seconds

        x = radius * np.cos(omega * elapsed_time)
        y = radius * np.sin(omega * elapsed_time)
        theta = omega * elapsed_time + np.pi/2

        #calculate actions:
        linear_velocity = radius * omega
        angular_velocity = omega

        #print(f'Publishing reference: x={x}, y={y}, theta={theta}, linear_velocity={linear_velocity}, angular_velocity={angular_velocity}')

        v = linear_velocity
        w = angular_velocity

        #publish as vector3 to the topic
        cmd = Vector3()
        cmd.x = v
        cmd.y = w
        cmd.z = 0.0  #not used
        #publish actions to the topic as cmd_vel
        self.cmd_vel_pub.publish(cmd)
    
    def poses_callback(self, msg):
        """Store latest mocap poses for closed-loop control"""
        # Update dictionary with latest poses
        for pose in msg.poses:
            self.latest_poses[pose.name] = pose.pose
        
        # Log only occasionally to avoid spam
        if len(self.latest_poses) > 0 and not hasattr(self, '_poses_logged'):
            self._poses_logged = True
            robot_names = ', '.join(self.latest_poses.keys())
            self.get_logger().info(f'Receiving mocap data for robots: {robot_names}')


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceTrajectoryPublisher()
    node.get_logger().info('=== MAIN FUNCTION REACHED, STARTING SPIN ===')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np
import sys
import os

# Add wmr-simulator scripts to path
# When installed, the scripts should be in share/wmr_controller/deps/wmr-simulator/scripts
import ament_index_python.packages
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    wmr_sim_path = os.path.join(package_share_directory, 'deps/wmr-simulator/scripts')
except Exception:
    wmr_sim_path = None

# If not found in share (e.g. not installed yet or running from source differently), try relative path
if not wmr_sim_path or not os.path.exists(wmr_sim_path):
     # When running from source: src/wmr_controller/wmr_controller/wmr_controller_node.py
     # Target: src/wmr_controller/deps/wmr-simulator/scripts
     wmr_sim_path = os.path.join(os.path.dirname(__file__), '../deps/wmr-simulator/scripts')

if os.path.exists(wmr_sim_path):
    sys.path.insert(0, wmr_sim_path)
    # from controller import Controller  (Unused in plan)
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")




class CMCGSPlanNode(Node):
    def __init__(self):
        super().__init__('cmcgs_plan')
        
        # Parameters
        self.declare_parameter('robot_name', 'cf10')
        self.declare_parameter('plan_tolerance', 0.05)  # 5cm tolerance for reaching waypoint

        self.robot_name = self.get_parameter('robot_name').value
        self.plan_tolerance = self.get_parameter('plan_tolerance').value
        
        # ROS Communication
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.mocap_sub = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            mocap_qos
        )
        
        # Plan publisher - keep only the latest plan (size 1)
        self.plan_pub = self.create_publisher(Pose2D, 'plan', 1)
        
        # Planning State
        self.current_plan = [] # List of Pose2D waypoints
        self.current_goal_index = 0
        self.latest_pose = None
        self.initialized = False
        
        # Timer for planning loop (10Hz)
        self.timer = self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info(f'CMCGS Planner started for robot: {self.robot_name}')

    def generate_plan(self):
        """
        Placeholder planning function.
        Returns a list of Pose2D waypoints.
        """
        plan = []
        # Example: Square pattern
        wps = [
            (0.5, 0.0, 0.0),
            (0.5, 0.5, 1.57),
            (0.0, 0.5, 3.14),
            (0.0, 0.0, -1.57)
        ]
        
        for wp in wps:
            p = Pose2D()
            p.x = wp[0]
            p.y = wp[1]
            p.theta = wp[2]
            plan.append(p)
            
        return plan

    def planning_loop(self):
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            return

        # Replan every loop
        self.current_plan = self.generate_plan()

        # Get current goal
        goal_pose = self.current_plan[0]
        
        # Publish current goal
        self.plan_pub.publish(goal_pose)

        # Check if reached
        dist = np.sqrt((self.latest_pose.position.x - goal_pose.x)**2 + 
                       (self.latest_pose.position.y - goal_pose.y)**2)
        
        if dist < self.plan_tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_goal_index}: ({goal_pose.x:.2f}, {goal_pose.y:.2f})")
            
            # Move to next waypoint
            self.current_goal_index += 1

    def poses_callback(self, msg: NamedPoseArray):
        """Callback for motion capture poses"""
        for pose in msg.poses:
            if pose.name == self.robot_name:
                self.latest_pose = pose.pose
                if not self.initialized:
                    self.initialized = True
                    self.get_logger().info('Planner received first pose')
                break

def main(args=None):
    rclpy.init(args=args)
    node = CMCGSPlanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

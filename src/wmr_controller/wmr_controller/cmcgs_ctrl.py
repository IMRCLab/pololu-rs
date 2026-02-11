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
    from controller import Controller
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")




class CMCGSCtrlNode(Node):
    def __init__(self):
        super().__init__('cmcgs_ctrl')
        
        # Parameters
        self.declare_parameter('robot_name', 'cf10')
        self.declare_parameter('frequency', 10)  # match control action frequency

        self.robot_name = self.get_parameter('robot_name').value
        frequency = self.get_parameter('frequency').value
        self.dt = 1.0 / frequency
        
        # Robot parameters for pololu robots
        self.robot_param = {
            'wheel_radius': 0.0165,      # 16.5mm wheel radius
            'base_diameter': 0.085        # 85mm wheelbase
        }
        
        # Initialize controller from wmr-simulator
        controller_gains = [5, 2, 2, 1.0, 1.0, 0.1, 0.1]  # [kx, ky, kth, kpr, kpl, kir, kil]
        cmd_limits = (-10.0, 10.0)  # Wheel speed limits (rad/s) for pololu robots (seems to be lower than actual wheelspeed limits)
        self.controller = Controller(self.robot_param, gains=controller_gains, 
                                    cmd_limits=cmd_limits, dt=self.dt)
        
        # QoS for mocap (BEST_EFFORT like controller_interface)
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for control commands (BEST_EFFORT, depth=1)
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to mocap poses
        self.mocap_sub = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            mocap_qos
        )

        # Plan subscription - keep only the latest plan (size 1)
        self.latest_plan = None
        self.plan_sub = self.create_subscription(
            Pose2D,
            'plan',
            self.plan_callback,
            1
        )
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.cmd_pub = self.create_publisher(Vector3, '/cmd_unicycle', cmd_qos)
        
        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # State
        self.latest_pose = None
        self.initialized = False
        self.wheel_speeds = (0.0, 0.0)  # Estimated wheel speeds (ur, ul)
        
        self.get_logger().info(f'ANN-CMCGS Node started for robot: {self.robot_name} @ {frequency} Hz')
    
    
    def poses_callback(self, msg: NamedPoseArray):
        """Callback for motion capture poses"""

        #idea: mocap publishes at its own rate. The last pose that was incoming is stored for usage at the defined control loop execution rate.
        for pose in msg.poses:
            if pose.name == self.robot_name:
                self.latest_pose = pose.pose

                #init pose. Take zero if mocap doesn't have a pose yet.
                if not self.initialized:
                    x0 = pose.pose.position.x
                    y0 = pose.pose.position.y
                    # extract yaw from quaternion
                    q = pose.pose.orientation
                    th0 = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
                    self.initialized = True
                    self.get_logger().info(f'Initialized at x={x0:.3f}, y={y0:.3f}, theta={th0:.3f}')
                break
    
    def plan_callback(self, msg: Pose2D):
        self.latest_plan = msg

    def control_loop(self):
        """copied from simulator.py and adapted to run inside a ros2 node """
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            return

        # Use latest plan if available, otherwise stand still
        
        #get true pose from mocap (replaces robot.get_pose() in simulator)
        x_true = self.latest_pose.position.x
        y_true = self.latest_pose.position.y
        q = self.latest_pose.orientation
        theta_true = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        pose_true = (x_true, y_true, theta_true)
        
        #get true wheel speeds (in simulator: robot.get_wheel_speeds())
        #TODO: get robot log data eventually to use encoder readings for real wheel speeds
        #for now: wheel speed command is assumed to be true
        ur_true, ul_true = self.wheel_speeds
        
        t = self.get_clock().now().nanoseconds / 1e9

        if self.latest_plan is not None:
             px_d = self.latest_plan.x
             py_d = self.latest_plan.y
             th_d = self.latest_plan.theta
             
             # Zero derivatives if not provided (assume static target point for now)
             vx_d = 0.0
             vy_d = 0.0
             w_d = 0.0
             ax_d = 0.0
             ay_d = 0.0
        else:
            # If no plan, stay put at current location (or stop)
            # We'll set desired state to current state to minimize error -> zero control
            px_d = x_true
            py_d = y_true
            th_d = theta_true
            vx_d = 0.0
            vy_d = 0.0
            w_d = 0.0
            ax_d = 0.0
            ay_d = 0.0
        
        ref_state = [px_d, py_d, th_d, vx_d, vy_d, w_d, ax_d, ay_d]
        
        #compute control commands using estimated states, use mocap pose as "true" pose instead of estimating it
        # We skip the internal wheel speed PID of the Controller class because our robot has onboard low-level control.
        # We only use the pose control part to get desired wheel speeds.
        ur_cmd, ul_cmd = self.controller._pose_control(ref_state, pose_true)
        
        #convert wheel speeds to (v, w) for publishing
        #TODO: is it maybe better to publish r & l for pololu?
        r = self.robot_param['wheel_radius']
        L = self.robot_param['base_diameter']
        v = r * (ur_cmd + ul_cmd) / 2.0
        w = r * (ur_cmd - ul_cmd) / L
        
        #publish control actions --> controller interface expects (v, w) and sends it to pololu like x box controller inputs
        cmd = Vector3()
        cmd.x = v
        cmd.y = w
        cmd.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Store commanded wheel speeds as "true" for next iteration
        # (in simulator, robot.step() updates these, i simply use commanded values here)
        self.wheel_speeds = (ur_cmd, ul_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CMCGSCtrlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

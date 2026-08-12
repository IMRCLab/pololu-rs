#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np
import sys
import os
import time
import glob
import yaml

venv_pattern = os.path.expanduser('~/wmr-ros/ROS/.venv/lib/python3.*/site-packages')
venv_matches = glob.glob(venv_pattern)
if venv_matches:
    venv_path = venv_matches[0]
    if venv_path not in sys.path:
        sys.path.insert(0, venv_path)

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
    # from controller import Controller
    from estimator import DiffDriveEstimator
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")

mpc_path = None
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    mpc_path = os.path.join(package_share_directory, 'external/realtime-dbastar/baselines/wmr-simulator/scripts')
except Exception:
    pass

if not mpc_path or not os.path.exists(mpc_path):
    mpc_path = os.path.join(os.path.dirname(__file__), '../external/realtime-dbastar/baselines/wmr-simulator/scripts')

if os.path.exists(mpc_path):
    sys.path.insert(0, mpc_path)

    from mpcController import QTO_MPC
    from simulator import goal_reached
else:
    print(f"Error: Could not find scripts at {mpc_path}")


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu09')
        self.declare_parameter('frequency', 10)  # match control action frequency
        self.robot_name = self.get_parameter('robot_name').value
        frequency = self.get_parameter('frequency').value
        # self.dt = 1.0 / frequency # TODO check if estimator is needed / can run at 100Hz as in MPC config
        self.declare_parameter('problem', None)

        problem_path = self.get_parameter('problem').value
        self.problem = yaml.safe_load(open(problem_path, 'r'))

        # Robot parameters for pololu robots
        self.robot_param = {
            'wheel_radius': 0.0165,      # 16.5mm wheel radius
            'base_diameter': 0.085        # 85mm wheelbase
        }
        
        # Initialize controller from mpcController
        self.controller = QTO_MPC(robot_param=self.problem)
        self.controller.initiaize_variables()

        self.start = list(self.problem["start"])
        self.goal = list(self.problem["goal"])

        self.dt, self.thr = float(self.problem["dbastar"]["dt"]), float(self.problem["dbastar"]["goal_threshold"])
        self.steps_max = int(self.problem["sim_time"] / self.dt)
        self.steps = 0

        self.t_compute, self.reached = 0.0, False
        self.traj = []
        self.log_wheel_cmd = []

        #init estimator from wmr-simulator
        estimator_cfg = {
            "type": "dr",  # Dead reckoning for now (or "kf" for Kalman filter)
            "wheel_radius": self.robot_param['wheel_radius'],
            "base_diameter": self.robot_param['base_diameter'],
            "start": [0.0, 0.0, 0.0],  # Will be updated from first mocap pose
            "noise_pos": 0.001,
            "noise_angle": 0.01,
            "enc_angle_noise": 0.0,
            "proc_pos_std": 0.01,
            "proc_theta_std": 0.01
        }
        self.estimator = DiffDriveEstimator(estimator_cfg, self.dt)
        
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
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.cmd_pub = self.create_publisher(Vector3, '/cmd_unicycle', cmd_qos)
        
        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # State
        self.latest_pose = None
        self.initialized = False
        self.wheel_speeds = (0.0, 0.0)  # Estimated wheel speeds (ur, ul)
        
        self.get_logger().info(f'WMR Controller started for robot: {self.robot_name} @ {frequency} Hz')
    
    
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
                    self.estimator._init_state(x0, y0, th0)
                    self.initialized = True
                    self.get_logger().info(f'Initialized at x={x0:.3f}, y={y0:.3f}, theta={th0:.3f}')
                break
    
    def control_loop(self):
        """copied from simulator.py and adapted to run inside a ros2 node """
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            return
        
        #get true pose from mocap (replaces robot.get_pose() in simulator)
        x_true = self.latest_pose.position.x
        y_true = self.latest_pose.position.y
        q = self.latest_pose.orientation
        theta_true = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        pose_true = (x_true, y_true, theta_true)
        self.traj.append(pose_true)

       
        
        # stop if goal reached 
        if goal_reached(pose_true, self.goal, self.thr, float(self.problem["dbastar"]["goal_error_tolerance"])):
            cmd = Vector3()
            self.cmd_pub.publish(cmd)
            reached = True
            self.get_logger().info(f'v={cmd.x:.3f}, w={cmd.y:.3f}')
            cost_s = (len(self.traj) - 1) * self.dt if self.reached else None
            controls = np.array(self.log_wheel_cmd)
            # self.get_logger().info(
            #     f"reached: {self.reached}, cost_s: {cost_s}, "
            #     f"search_time_s: {self.t_compute}, traj: {self.traj}, "
            #     f"controls: {controls}, dt: {self.dt}"
            # )
            return
        
         #   stop if max steps reached
        if self.steps > self.steps_max:
            cmd = Vector3()
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'v={cmd.x:.3f}, w={cmd.y:.3f}')

            cost_s = (len(self.traj) - 1) * self.dt if self.reached else None
            controls = np.array(self.log_wheel_cmd)
            # self.get_logger().info(
            #     f"reached: {self.reached}, cost_s: {cost_s}, "
            #     f"search_time_s: {self.t_compute}, traj: {self.traj}, "
            #     f"controls: {controls}, dt: {self.dt}"
            # )
            return
        #get true wheel speeds (in simulator: robot.get_wheel_speeds())
        #TODO: get robot log data eventually to use encoder readings for real wheel speeds
        #for now: wheel speed command is assumed to be true
        ur_true, ul_true = self.wheel_speeds
        
        #self.estimator.update(ur_true, ul_true, pose_true)
        
        #pose_est = self.estimator.get_est_pose()  # (x_hat, y_hat, theta_hat)
        ur_hat, ul_hat = self.estimator.get_est_wheel_speeds()
        wheel_est = (ur_hat, ul_hat)

        t0 = time.perf_counter()
        #compute control commands using estimated states, use mocap pose as "true" pose instead of estimating it
        ul_cmd, ur_cmd = self.controller.solve(list(pose_true), list(self.goal))
        self.t_compute += time.perf_counter() - t0

        
        #convert wheel speeds to (v, w) for publishing
        #TODO: is it maybe better to publish r & l for pololu?
        r = self.robot_param['wheel_radius']
        L = self.robot_param['base_diameter']
        v = r * (ur_cmd + ul_cmd) / 2.0
        w = r * (ur_cmd - ul_cmd) / L
        # self.get_logger().info(f'v={v:.3f}, w={w:.3f}')

        #publish control actions --> controller interface expects (v, w) and sends it to pololu like x box controller inputs
        cmd = Vector3()
        cmd.x = v
        cmd.y = w
        cmd.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Store commanded wheel speeds as "true" for next iteration
        # (in simulator, robot.step() updates these, i simply use commanded values here)
        self.wheel_speeds = (ur_cmd, ul_cmd)
        self.log_wheel_cmd.append(self.wheel_speeds)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

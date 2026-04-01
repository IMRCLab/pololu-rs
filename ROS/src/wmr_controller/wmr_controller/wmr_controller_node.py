#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3
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
    from estimator import DiffDriveEstimator
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")




class WMRControllerNode(Node):
    def __init__(self):
        super().__init__('wmr_controller_node')
        
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
        #self.estimator = DiffDriveEstimator(estimator_cfg, self.dt)
        
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
        
        #get true wheel speeds (in simulator: robot.get_wheel_speeds())
        #TODO: get robot log data eventually to use encoder readings for real wheel speeds
        #for now: wheel speed command is assumed to be true
        ur_true, ul_true = self.wheel_speeds
        
        #self.estimator.update(ur_true, ul_true, pose_true)
        
        #pose_est = self.estimator.get_est_pose()  # (x_hat, y_hat, theta_hat)
        ur_hat, ul_hat = self.estimator.get_est_wheel_speeds()
        wheel_est = (ur_hat, ul_hat)
        
        t = self.get_clock().now().nanoseconds / 1e9

        #get a dummy reference trajectory calculation here
        #TODO: adapt to problem definition to read in from yaml 
        radius = 0.3
        omega = 0.71
        px_d = radius * np.cos(omega * t)
        py_d = radius * np.sin(omega * t)
        th_d = omega * t + np.pi/2
        vx_d = -radius * omega * np.sin(omega * t)
        vy_d = radius * omega * np.cos(omega * t)
        w_d = omega
        ax_d = -radius * omega**2 * np.cos(omega * t)
        ay_d = -radius * omega**2 * np.sin(omega * t)
        
        ref_state = [px_d, py_d, th_d, vx_d, vy_d, w_d, ax_d, ay_d]
        
        #compute control commands using estimated states, use mocap pose as "true" pose instead of estimating it
        ur_cmd, ul_cmd = self.controller.compute(ref_state, pose_true, wheel_est)
        
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

        
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--problem", type=str, default="problems/empty.yaml", help="path to problem and robot description yaml file")
    # parser.add_argument("--output", type=str, default="simulation_visualization", help="path to output visualization pdf and html file")
    # args = parser.parse_args()

    # # Initialize robot model
    # problem_path = args.problem
    # with open(problem_path, 'r') as file:
    #     problem = yaml.safe_load(file)

    # # Simulation parameters
    # sim_time = problem["sim_time"]  # simulation duration (s)
    # dt = float(problem["time_step"])
    # sim_N = int(sim_time / dt)
    # sim_time_grid = np.linspace(0, sim_N * dt, sim_N + 1)
    # start = problem["start"]
    # goal = problem["goal"]

    # # Initialize robot and estimator
    # robot_cfg = problem["robot"]
    # robot = DiffDrive(robot_cfg=robot_cfg, init_state=start, dt=dt)
    # # Planning time horizon
    # planner_cfg = problem["planner"]
    # planner_time = planner_cfg["time"]  # total time for reference trajectory (s)
    # planner_N = int(planner_time / dt)
    # planner_time_grid = np.linspace(0, planner_N * dt, planner_N + 1)
    
    # # Generate reference trajectory over full planner horizon
    # waypoints = planner_cfg["waypoints"]
    # reference_states, polynomial_traj = compute_reference_trajectory(start, goal, waypoints, planner_time_grid)

    # # Initialize estimator
    # est_cfg = problem["estimator"]  # may be empty
    # # Estimator uses robot+estimator params
    # estimator = DiffDriveEstimator(estimator_cfg=est_cfg,
    #                             dt=dt)    # simulation time horizon
    
    # # Initialize controller
    # ctrl_cfg = problem["controller"]
    # ctrl = Controller(robot_param=est_cfg,
    #                   gains=ctrl_cfg["gains"],
    #                   cmd_limits=[-robot_cfg["max_wheel_speed"], robot_cfg["max_wheel_speed"]],
    #                   dt=dt)    
    
    # # Simulate only for the simulation time horizon
    # for k in range(len(sim_time_grid)):
    #     # Update estimator with true wheel speeds
    #     ur_true, ul_true = robot.get_wheel_speeds()
    #     pose_true = robot.get_pose()
    #     estimator.update(ur_true, ul_true, pose_true)
    #     pose_est = estimator.get_est_pose()
    #     ur_hat, ul_hat = estimator.get_est_wheel_speeds()
    #     wheel_est = (ur_hat, ul_hat)
    #     # Determine which reference state to use
    #     if k < len(reference_states):
    #         # Use current reference state if available
    #         ref_state = reference_states[k]
    #     else:
    #         # Use last available reference state if simulation continues beyond planner time
    #         ref_state = reference_states[-1]

    #     # Compute control commands [ur_cmd, ul_cmd] using reference at current time step
    #     u = ctrl.compute(ref_state, pose_est, wheel_est)

    #     # Step the robot simulation
    #     robot.step(u)

def main(args=None):
    rclpy.init(args=args)
    node = WMRControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

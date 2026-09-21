#!/usr/bin/env python3
import copy
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, Vector3
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from wmr_interfaces.msg import AABB2DArray
from wmr_controller.aabb_utils import (
    aabb_map_from_messages,
    compare_aabb_maps,
    planner_obstacles,
)
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
    from controller import Controller
    from estimator import DiffDriveEstimator
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")

mpc_dbastar_path = None
default_problem_path = None
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    mpc_dbastar_path = os.path.join(package_share_directory, 'external/realtime-dbastar/baselines/wmr-simulator/scripts')
    default_problem_path = os.path.join(package_share_directory, 'external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml')
except Exception:
    pass

if not mpc_dbastar_path or not os.path.exists(mpc_dbastar_path):
    mpc_dbastar_path = os.path.join(os.path.dirname(__file__), '../external/realtime-dbastar/baselines/wmr-simulator/scripts')
if not default_problem_path or not os.path.exists(default_problem_path):
    default_problem_path = os.path.join(os.path.dirname(__file__), '../external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml')

if not os.path.exists(default_problem_path):
    print(f"Error: Could not find problem file at {default_problem_path}")

if os.path.exists(mpc_dbastar_path):
    sys.path.insert(0, mpc_dbastar_path)
    from benchmark import _deep_merge
    from simulator import goal_reached, obstacles_of, dynamic_obstacles_of, vanishing_obstacles_of, displacements_of, _mpc_sized_for
    from mpcController import QTO_MPC
else:
    print(f"Error: Could not find scripts at {mpc_dbastar_path}")

traj_out_path = '/home/lndw/wmr-ros/ROS/src/wmr_controller/wmr_controller/results/mpc/'
class mpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu09')
        self.declare_parameter('mocap_topic', '/poses')
        self.declare_parameter('cmd_unicycle_topic', '/cmd_unicycle')
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('problem', default_problem_path)
        self.declare_parameter('instance', '1.3_0.5_2.3562_empty')
        self.declare_parameter('obstacle_topic', '/obstacles_aabb')
        self.declare_parameter('obstacle_change_tolerance', 0.02)
        self.declare_parameter('max_obstacles', 15)  # Maximum number of obstacles for MPC

        self.robot_name = str(self.get_parameter('robot_name').value)
        mocap_topic = str(self.get_parameter('mocap_topic').value)
        cmd_unicycle_topic = str(self.get_parameter('cmd_unicycle_topic').value)
        control_dt = float(self.get_parameter('control_dt').value)
        problem_path = str(self.get_parameter('problem').value)
        instance_name = str(self.get_parameter('instance').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.obstacle_change_tolerance = float(
            self.get_parameter('obstacle_change_tolerance').value
        )
        self.max_obstacles = int(self.get_parameter('max_obstacles').value)


        if control_dt <= 0.0:
            raise ValueError("ROS parameter 'control_dt' must be greater than zero")
        if not os.path.isfile(problem_path):
            raise FileNotFoundError(f"Problem file does not exist: {problem_path}")
        if self.obstacle_change_tolerance < 0.0:
            raise ValueError("obstacle_change_tolerance must be non-negative")

        with open(problem_path, 'r') as f:
            cfg = yaml.safe_load(f)
        shared = {k: v for k, v in cfg.items() if k != "instances"}
        out = {}
        for inst in cfg.get("instances") or []:
            inst = dict(inst)
            name = str(inst.pop("name"))
            # Environment tweaks written inline next to the goal (obstacles/min/max),
            # or a full `environment:` dict, layered over the shared environment.
            inline_env = dict(inst.pop("environment", {}) or {})
            for k in ("min", "max", "obstacles", "dynamic_obstacles", "vanishing_obstacles",
                    "displacements"):
                if k in inst:
                    inline_env[k] = inst.pop(k)
            prob = _deep_merge(shared, inst)             # inst now carries goal (+ dbastar/etc overrides)
            prob["environment"] = _deep_merge(cfg.get("environment") or {}, inline_env)
            out[name] = prob
        
        if instance_name not in out:
            available = ", ".join(out.keys())
            raise ValueError(
                f"Unknown benchmark instance '{instance_name}'. Available: {available}"
            )
        problem = out[instance_name]
        
        self.controller_dt = float(problem["dbastar"]["dt"])
        if self.controller_dt <= 0.0:
            raise ValueError(
                "dbastar.dt must be greater than zero"
            )

        self.thr = float(problem["dbastar"]["goal_threshold"])
        self.steps_max = int(problem["sim_time"] / self.controller_dt)

        
        self.problem = problem
        self.start = list(self.problem["start"])
        self.goal = list(self.problem["goal"])

        self.setpoints = np.asarray([], dtype=float).reshape(0, 6)  # [x, y, theta, vx, vy, w]

        self.i = 0

        self.traj = []
        self.log_wheel_cmd = []

         # QoS for mocap (BEST_EFFORT like controller_interface)
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
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
            mocap_topic,
            self.poses_callback,
            mocap_qos
        )

        obs_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.obstacle_sub = self.create_subscription(
            AABB2DArray,
            obstacle_topic,
            self.obstacles_callback,
            obs_qos
        )
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.cmd_pub = self.create_publisher(Vector3, cmd_unicycle_topic, cmd_qos)
        
        # Timer for control loop
        self.timer = self.create_timer(self.controller_dt, self.control_loop)     

        # Robot parameters for pololu robots
        robot_cfg = self.problem["robotcfg"]
        self.robot_param = {
            'wheel_radius':
                float(robot_cfg["wheel_radius"]),
            'base_diameter':
                float(robot_cfg["base_diameter"]),
        }

        self.get_logger().info("Initializing MPC controller...")
        self.ctrl = QTO_MPC(robot_param=_mpc_sized_for(problem, self.max_obstacles))
        self.ctrl.initiaize_variables()

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
        self.estimator = DiffDriveEstimator(estimator_cfg, self.controller_dt)

        # AABB snapshots are reusable ROS inputs.  Planning stays single-threaded
        # because every SMAG invocation writes the same traj/*.csv files.
        self.obstacles_initialized = False
        self.observed_obstacles = {}
        # self.observed_obstacles = {
        #     f"yaml_{index}": tuple(box)
        #     for index, box in enumerate(obstacles_of(self.problem))
        # }
       
        # State
        self.latest_pose = None
        self.initialized = False
        self.wheel_speeds = (0.0, 0.0)  # Estimated wheel speeds (ur, ul)
        
        self.traj_out_path = os.path.join(traj_out_path, f'traj_{instance_name}.csv')

        self.get_logger().info(
            f'MPC controller started for robot: {self.robot_name} @ {1.0/self.controller_dt:g} Hz; '
            f'mocap={mocap_topic}, obstacles={obstacle_topic}, '
            f'cmd={cmd_unicycle_topic}, instance={instance_name}'
        )
    

    def stop_robot(self):
        self.cmd_pub.publish(Vector3())
        self.timer.cancel()
        self.stopped = True
    
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
                    th0 = self.estimator._wrap_to_pi(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
                    self.estimator._init_state(x0, y0, th0)
                    self.initialized = True
                    self.get_logger().info(f'Initialized at x={x0:.3f}, y={y0:.3f}, theta={th0:.3f}')
                    # self.get_logger().info(f'Initialized_Flag_Pose_Callback: {self.initialized}')
                break


    def current_robot_pose(self):
        if self.latest_pose is None:
            return None

        pose = self.latest_pose
        q = pose.orientation
        theta = self.estimator._wrap_to_pi(
            np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
            )
        )
        return [float(pose.position.x), float(pose.position.y), float(theta)]

    def obstacles_callback(self, msg: AABB2DArray):
        """Consume a complete, current AABB snapshot."""
        self.get_logger().warn('obstacle_callback is called')
        if not self.obstacles_initialized:
            self.observed_obstacles = aabb_map_from_messages(msg.boxes)
            self.obstacles_initialized = True
        else:
            try:
                new_obstacles = aabb_map_from_messages(msg.boxes)
                changes = compare_aabb_maps(
                    self.observed_obstacles,
                    new_obstacles,
                    self.obstacle_change_tolerance,
                )
            except ValueError as error:
                self.get_logger().warn(f"Ignoring invalid AABB snapshot: {error}")
                return

            if not changes.changed:
                self.get_logger().info("AABB map unchanged")
                return

            self.observed_obstacles = new_obstacles
            self.cmd_pub.publish(Vector3())
            self.get_logger().info(
                f"AABB map changed: added={list(changes.added)}, "
                f"removed={list(changes.removed)}, moved={list(changes.moved)}"
            )

    
    def control_loop(self):
        """copied from simulator.py and adapted to run inside a ros2 node """
        # self.get_logger().info(f'Initialized_Flag_Control_Loop: {self.initialized}')
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Vector3())
            return
        
        if not self.obstacles_initialized:
            self.get_logger().warn('Waiting for first obstacle map...', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Vector3())
            return
        
        #get true pose from mocap (replaces robot.get_pose() in simulator)
        x_true = self.latest_pose.position.x
        y_true = self.latest_pose.position.y
        q = self.latest_pose.orientation
        theta_true = self.estimator._wrap_to_pi(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
        pose_true = (x_true, y_true, theta_true)
        self.traj.append(pose_true)

        self.reached = goal_reached(pose_true, self.goal, self.thr, float(self.problem["dbastar"]["goal_error_tolerance"]))
        # stop if goal reached
        if self.reached:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            return

        if self.i >= self.steps_max:
            self.get_logger().warn(
                "Maximum control duration reached"
            )
            self.stop_robot()
            return
        
        #get true wheel speeds (in simulator: robot.get_wheel_speeds())
        #TODO: get robot log data eventually to use encoder readings for real wheel speeds
        #for now: wheel speed command is assumed to be true
        # ur_true, ul_true = self.wheel_speeds
        
        #self.estimator.update(ur_true, ul_true, pose_true)
        
        #pose_est = self.estimator.get_est_pose()  # (x_hat, y_hat, theta_hat)
        # ur_hat, ul_hat = self.estimator.get_est_wheel_speeds()
        # wheel_est = (ur_hat, ul_hat)

        #compute control commands using estimated states, use mocap pose as "true" pose instead of estimating it
        # ref_state = self.setpoints[self.i]
        # self.get_logger().info('Solving MPC...')
        boxes = planner_obstacles(self.observed_obstacles)
        obs_p = self.ctrl.pack_obstacles(boxes)

        ul_cmd, ur_cmd = self.ctrl.solve(
            list(pose_true),
            list(self.goal),
            obs=obs_p,
)
        ul_cmd, ur_cmd = self.ctrl.solve(list(pose_true), list(self.goal), obs=obs_p) 
        self.i += 1
        
        #convert wheel speeds to (v, w) for publishing
        #TODO: is it maybe better to publish r & l for pololu?
        r = self.robot_param['wheel_radius']
        L = self.robot_param['base_diameter']
        v = r * (ur_cmd + ul_cmd) / 2.0
        w = r * (ur_cmd - ul_cmd) / L
        # self.get_logger().info(f'v={v:.3f}, w={w:.3f}')
        # self.get_logger().info(f'x={ref_state[0]:.3f}, y={ref_state[1]:.3f}, theta={ref_state[2]:.3f}')
        # self.get_logger().info(f'ur={ur_cmd:.3f}, ul={ul_cmd:.3f}')

        #publish control actions --> controller interface expects (v, w) and sends it to pololu like x box controller inputs
        cmd = Vector3()
        cmd.x = v
        cmd.y = w
        cmd.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Store commanded wheel speeds as "true" for next iteration
        # (in simulator, robot.step() updates these, i simply use commanded values here)
        self.wheel_speeds = (ul_cmd, ur_cmd)
        self.log_wheel_cmd.append(self.wheel_speeds)


def main(args=None):
    rclpy.init(args=args)
    node = mpcControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # with open(node.traj_out_path, 'w') as f:
        #     f.write("x,y,theta\n")
        #     for pose in node.traj:
        #         f.write(f"{pose[0]:.6f},{pose[1]:.6f},{pose[2]:.6f}\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
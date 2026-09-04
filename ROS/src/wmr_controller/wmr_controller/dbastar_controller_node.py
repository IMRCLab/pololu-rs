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
BENCHMARK_FILE = "/home/lndw/wmr-ros/ROS/src/wmr_controller/external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml"
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
    from benchmark import _deep_merge
    from simulator import _run_smag_once, _smag_plan, goal_reached, obstacles_of, dynamic_obstacles_of, vanishing_obstacles_of, displacements_of, _mpc_sized_for, triggered, displacement_triggered, in_collision
else:
    print(f"Error: Could not find scripts at {mpc_path}")


class dbastarControllerNode(Node):
    def __init__(self):
        super().__init__('dbastar_controller_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu09')
        self.declare_parameter('mocap_topic', '/poses')
        self.declare_parameter('cmd_unicycle_topic', '/cmd_unicycle')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('problem', BENCHMARK_FILE)
        self.declare_parameter('instance', '-1.5_-2.5_-0.7854')

        self.robot_name = str(self.get_parameter('robot_name').value)
        mocap_topic = str(self.get_parameter('mocap_topic').value)
        cmd_unicycle_topic = str(self.get_parameter('cmd_unicycle_topic').value)
        frequency = float(self.get_parameter('control_rate').value)
        problem_path = str(self.get_parameter('problem').value)
        instance_name = str(self.get_parameter('instance').value)

        if frequency <= 0.0:
            raise ValueError("ROS parameter 'control_rate' must be greater than zero")
        if not os.path.isfile(problem_path):
            raise FileNotFoundError(f"Problem file does not exist: {problem_path}")

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
        
        configured_dt = float(problem["dbastar"]["dt"])
        self.dt = 1.0 / frequency
        if not np.isclose(self.dt, configured_dt):
            self.get_logger().warn(
                f"control_rate={frequency:g} Hz overrides dbastar.dt="
                f"{configured_dt:g} s with dt={self.dt:g} s"
            )
            problem["dbastar"]["dt"] = self.dt
        self.thr = float(problem["dbastar"]["goal_threshold"])
        self.steps_max = int(problem["sim_time"] / self.dt)
        static = obstacles_of(problem)
        dyn, van = dynamic_obstacles_of(problem), vanishing_obstacles_of(problem)
        self.shoves = list(displacements_of(problem))
        self.known = list(static) + [v["box"] for v in van]
        self.hidden, self.present = list(dyn), list(van)
        # self.obs_p = self.ctrl.pack_obstacles(self.known) if self.ctrl.M else None
        self.t_compute, self.reached, self.reveals, self.vanishes = 0.0, False, [], []
        self.shifts, self.jumps = [], []
        

        self.problem = problem
        self.start = list(self.problem["start"])
        self.goal = list(self.problem["goal"])

        self.i = 0

        self.traj = []
        self.log_wheel_cmd = []

        self.get_logger().info("Running planner once")
        _run_smag_once(self.problem, self.start, self.goal)
        self.get_logger().info("Planner finished, getting setpoints from csv")

        self.setpoints = _smag_plan()

        # Robot parameters for pololu robots
        robot_cfg = self.problem["robotcfg"]
        self.robot_param = {
            'wheel_radius':
                float(robot_cfg["wheel_radius"]),
            'base_diameter':
                float(robot_cfg["base_diameter"]),
        }

        #init controller from wmr-simulator
        dbastar_cfg = self.problem["dbastar"]
        self.kx = float(dbastar_cfg["kx"])
        self.ky = float(dbastar_cfg["ky"])
        self.kth = float(dbastar_cfg["kth"])
        self.get_logger().info(f'Controller gains: kx={self.kx}, ky={self.ky}, kth={self.kth}')
        controller_gains = [self.kx, self.ky, self.kth, 0.01, 0.01, 0.01, 0.01]  # [kx, ky, kth, kpr, kpl, kir, kil] 
        self.wheel_vel_upper_limits = float(robot_cfg.get("max_vel_leftwheel", 30.0))  # load wheel speed limits from benchmark.yaml
        self.wheel_vel_lower_limits = float(robot_cfg.get("min_vel_leftwheel", -30.0))  # load wheel speed limits from benchmark.yaml
        cmd_limits = (self.wheel_vel_lower_limits, self.wheel_vel_upper_limits)  # Wheel speed limits (rad/s) for pololu robots (seems to be lower than actual wheelspeed limits)
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
        self.estimator = DiffDriveEstimator(estimator_cfg, self.dt)
        

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
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.cmd_pub = self.create_publisher(Vector3, cmd_unicycle_topic, cmd_qos)
        
        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # State
        self.latest_pose = None
        self.initialized = False
        self.wheel_speeds = (0.0, 0.0)  # Estimated wheel speeds (ur, ul)
        
        self.get_logger().info(
            f'DBA* controller started for robot: {self.robot_name} @ {frequency:g} Hz; '
            f'mocap={mocap_topic}, cmd={cmd_unicycle_topic}, instance={instance_name}'
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
        theta_true = self.estimator._wrap_to_pi(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
        pose_true = (x_true, y_true, theta_true)
        self.traj.append(pose_true)

        self.reached = goal_reached(pose_true, self.goal, self.thr, float(self.problem["dbastar"]["goal_error_tolerance"]))
        # stop if goal reached or max iterations reached
        if self.reached or self.i > self.steps_max:
            cmd = Vector3()
            self.cmd_pub.publish(cmd)

            cost_s = (len(self.traj) - 1) * self.dt if self.reached else None
            controls = np.array(self.log_wheel_cmd)
            self.get_logger().info(
                f"reached: {self.reached}, cost_s: {cost_s}, "
                f"search_time_s: {self.t_compute}, traj: {self.traj}, "
                f"controls: {controls}, dt: {self.dt}"
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
        ref_state = self.setpoints[self.i]
        ref_state_full = np.asarray([
            ref_state[0],  # x
            ref_state[1],  # y
            ref_state[2],  # theta
            ref_state[3],  # vx
            ref_state[4],  # vy
            ref_state[5],  # w
            0.0,     # ax
            0.0,     # ay
        ])
        ur_cmd, ul_cmd = self.controller.compute(ref_state_full, pose_true, self.wheel_speeds) # this time it should really be ur, ul :D
        self.i +=1
        
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
    node = dbastarControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

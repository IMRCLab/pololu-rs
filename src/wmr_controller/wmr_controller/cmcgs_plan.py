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

# Add ann-cmcgs-async to path
ann_cmcgs_path = os.path.join(os.path.dirname(__file__), '../external/ann-cmcgs-async')
if os.path.exists(ann_cmcgs_path):
    sys.path.insert(0, ann_cmcgs_path)
    # Register envs
    import gymnasium as gym
    import tools.planners as planners
    from tools.planners import MCGSPlanner
    from tools.envs.navigation_envs.navigation_env_pololu import NavigationEnvPololu
    
    gym.register(
        id='NavigationEnvPololu-v0',
        entry_point='tools.envs.navigation_envs.navigation_env_pololu:NavigationEnvPololu',)
else:
    print(f"Error: Could not find ann-cmcgs-async at {ann_cmcgs_path}")


class ROSMCGSPlanner(MCGSPlanner):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def receive_latest_state(self, use_hardware: bool = True):
        if use_hardware:
            if self.node.latest_pose is None:
                # Fallback if no pose yet, though planner loop guards against this
                return np.zeros(4) 
            
            # Construct state [x, y, theta, time]
            x = self.node.latest_pose.position.x
            y = self.node.latest_pose.position.y
            
            q = self.node.latest_pose.orientation
            theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            
            # Using ROS time as the time dimension
            # Ensure it fits the scale expected by the planner (seconds)
            current_time = self.node.get_clock().now().nanoseconds / 1e9 - self.node.start_time
            
            new_state = np.array([x, y, theta, current_time], dtype=np.float32)
            # self.node.get_logger().info(f"ROS Planner State: {new_state}")
            return new_state
        else:
            return super().receive_latest_state(use_hardware=False)

    def publish_setpoint(self, setpoint_state, use_hardware: bool = True):
        if use_hardware:
            # setpoint_state is [x, y, theta, time]
            msg = Pose2D()
            msg.x = float(setpoint_state[0])
            msg.y = float(setpoint_state[1])
            msg.theta = float(setpoint_state[2])
            
            self.node.plan_pub.publish(msg)
            # self.node.get_logger().info(f"Published Plan Setpoint: {msg}")
        else:
            super().publish_setpoint(setpoint_state, use_hardware=False)

class CMCGSPlanNode(Node):
    def __init__(self):
        super().__init__('cmcgs_plan')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu08')
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
        self.latest_pose = None
        self.initialized = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Initialize Environment
        self.env = gym.make('NavigationEnvPololu-v0', 
               render_mode=None, # No rendering in ROS node
               dt=1.0,
               atol=0.3,
               rtol=0.0,
               multi_step_count=10,
               obstacle_mode="circles",
               render_sleep=0.0,
               sparse_reward=True,
        )
        self.env.reset(seed=42)

        # Initialize Planner
        computational_budget_max = 500
        time_budget_max = 0.2
        
        self.planner = ROSMCGSPlanner(
            node=self,
            env=self.env.unwrapped,
            budget_per_step=1,
            computational_budget_max=computational_budget_max,
            time_budget_max=time_budget_max,
            expand_n_times=1,
            sample_best_X_actions=5,
            kappa=0.5,
            alpha=0.2,
            k=30,
            radius_threshold=1.0,
            progressive_widening_method="dispersion",
            epsilon=0.1,
            c_uct=0.5,
            plan_in_space_time=False,
            use_controller=True,
            tracking_tolerance=0.5,
            yield_mode='N',
            random_rollout_n_times=1,
            random_rollout_length=4,
            force_exact_update=False,
            controller_expansion_check=False
        )
        # Bootstrap planning
        try:
             self.planner.reset() # Important to init the graph
             # self.planner.plan(iterations=20) 
        except Exception as e:
             self.get_logger().error(f"Error init planner: {e}")

        # Timer for planning loop (10Hz)
        self.timer = self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info(f'CMCGS Planner started for robot: {self.robot_name}')

    def planning_loop(self):
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            return

        # Use the MCGS Planner's online planning method
        # This will call receive_latest_state -> update_planner -> plan -> publish_setpoint
        try:
            self.planner.plan_online(
                computational=True,
                time=False,
                render=False,
                use_hardware=True 
            )
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

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

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
ann_cmcgs_path = None
try:
    # Try finding in share (installed mode)
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    ann_cmcgs_path = os.path.join(package_share_directory, 'external/ann-cmcgs-async')
except Exception:
    pass

if not ann_cmcgs_path or not os.path.exists(os.path.join(ann_cmcgs_path, 'tools')):
    # Try relative to source file (dev mode)
    ann_cmcgs_path = os.path.join(os.path.dirname(__file__), '../external/ann-cmcgs-async')

if os.path.exists(ann_cmcgs_path):
    sys.path.insert(0, ann_cmcgs_path)

    
    # We prioritize the local path vs openai baseline
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
            # Determine start state for the planner
            # If we have an active setpoint we are driving to, plan from there
            if self.node.active_setpoint_state is not None:
                state = self.node.active_setpoint_state.copy()
                # Update time to now (or projected arrival time?)
                # For now, using current time as the "start time" for the next plan segment
                current_time = self.node.get_clock().now().nanoseconds / 1e9 - self.node.start_time
                state[3] = current_time
                return state

            if self.node.latest_pose is None:
                return np.zeros(4) 
            
            # Construct state [x, y, theta, time]
            x = self.node.latest_pose.position.x
            y = self.node.latest_pose.position.y
            
            q = self.node.latest_pose.orientation
            theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            
            current_time = self.node.get_clock().now().nanoseconds / 1e9 - self.node.start_time
            
            new_state = np.array([x, y, theta, current_time], dtype=np.float32)
            return new_state
        else:
            return super().receive_latest_state(use_hardware=False)

    def publish_setpoint(self, setpoint_state, use_hardware: bool = True):
        if use_hardware:
            # Do NOT publish to ROS yet
            # Store as pending setpoint
            self.node.pending_setpoint_state = setpoint_state
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
        
        # Receding Horizon State
        self.active_setpoint_state = None  # The setpoint we are currently driving to
        self.pending_setpoint_state = None # The setpoint the planner just calculated

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

        # Check if we have a pending setpoint waiting to be activated
        if self.pending_setpoint_state is not None:
             # We have a plan ready.
             # Check if we should activate it (i.e. if we reached the previous active setpoint)
             
             activate = False
             if self.active_setpoint_state is None:
                 # First plan ever
                 activate = True
             else:
                 # Check distance to active setpoint
                 sp_x = self.active_setpoint_state[0]
                 sp_y = self.active_setpoint_state[1]
                 curr_x = self.latest_pose.position.x
                 curr_y = self.latest_pose.position.y
                 dist = np.sqrt((sp_x - curr_x)**2 + (sp_y - curr_y)**2)
                 
                 if dist <= self.plan_tolerance:
                     activate = True
                     # self.get_logger().info(f"reached waypoint, dist={dist:.3f}")
             
             if activate:
                 # Promote pending to active and publish
                 self.active_setpoint_state = self.pending_setpoint_state
                 self.pending_setpoint_state = None
                 
                 # Publish
                 msg = Pose2D()
                 msg.x = float(self.active_setpoint_state[0])
                 msg.y = float(self.active_setpoint_state[1])
                 msg.theta = float(self.active_setpoint_state[2])
                 self.plan_pub.publish(msg)
                 # self.get_logger().info(f"Published Plan: {msg.x:.2f}, {msg.y:.2f}")
                 
                 # Now pending is None, so we will fall through to planning below in the NEXT loop?
                 # Actually we can start planning immediately if we want continuous planning.
             else:
                 # Still driving to active setpoint, and we already have the NEXT one planned.
                 # So we just wait.
                 return

        # If we are here, pending_setpoint_state is None.
        # This means we need to plan the next step.
        # The planner will use active_setpoint_state as start if it exists (see receive_latest_state)
        
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

# set up planner
# plan one iteration (may take up to 1sec)
# take flag from controller interface, (if the controll follower was actualy started or not )only publish on /plan if the flag is set to true
# publish the first state as setpoint
# replan, take the setpoint as if was the actual robot position (so plan the next step)
# publish the new setpoint (only if pose matches setpoint)
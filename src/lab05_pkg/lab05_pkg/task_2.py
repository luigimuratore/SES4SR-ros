#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
dwa_path = os.path.join(current_dir, '../../planning_control_methods/Controllers/DWA')
sys.path.append(dwa_path)
from dwa import DWA
from utils import normalize, normalize_angle, calc_nearest_obs

class DWA_Enhanced(DWA):
    """
    Enhanced DWA with trapezoidal velocity profile:
    - Acceleration ramp: gradually increase speed
    - Cruise phase: maintain max speed
    - Deceleration ramp: gradually decrease to zero near goal
    
    J = α·heading + β·vel_profile + γ·dist_obst
    """
    
    def __init__(self, 
                 accel_distance=0.3,      # distance to reach max speed
                 decel_distance=0.4,      # distance to start slowing down
                 min_cruise_distance=0.2, # minimum distance for cruise phase
                 **kwargs):
        super().__init__(**kwargs)
        self.accel_distance = accel_distance
        self.decel_distance = decel_distance
        self.min_cruise_distance = min_cruise_distance
        self.initial_distance = None  # Store initial distance to goal
        
    def evaluate_paths(self, paths, velocities, goal_pose, robot_pose, obstacles):
        """
        Enhanced evaluation with trapezoidal velocity profile:
        J = α·heading + β·vel_profile + γ·obst_dist
        """
        # detect nearest obstacle
        nearest_obs = calc_nearest_obs(robot_pose, obstacles)

        # Compute the scores for the generated path
        # (1) heading_angle
        score_heading_angles = self.score_heading_angle(paths, goal_pose)
        # (2) velocity with trapezoidal profile
        score_vel = self.score_vel_trapezoidal(velocities, paths, goal_pose, robot_pose)
        # (3) obstacles
        score_obstacles = self.score_obstacles(paths, nearest_obs)

        # Scores Normalization
        score_heading_angles = normalize(score_heading_angles)
        score_vel = normalize(score_vel)
        score_obstacles = normalize(score_obstacles)

        # Compute the idx of the optimal path according to the overall score
        weights = np.array([[self.weight_angle, self.weight_vel, self.weight_obs]]).T
        scores = np.array([score_heading_angles, score_vel, score_obstacles])
        
        opt_idx = np.argmax(np.sum(scores * weights, axis=0))

        try:
            return opt_idx
        except:
            raise Exception("Not possible to find an optimal path")
    
    def score_vel_trapezoidal(self, u, path, goal_pose, robot_pose):
        """
        Trapezoidal velocity profile:
        
        1. Acceleration Phase (0 → accel_distance):
           v_desired = (dist_traveled / accel_distance) * v_max
           
        2. Cruise Phase (accel_distance → total_dist - decel_distance):
           v_desired = v_max
           
        3. Deceleration Phase (total_dist - decel_distance → goal):
           v_desired = (dist_to_goal / decel_distance) * v_max
        """
        vel = u[:, 0]  # Linear velocities of all trajectories
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)
        
        max_v = getattr(self.robot, "max_lin_vel", 0.22)
        
        # Calculate total distance (set initial distance once)
        if self.initial_distance is None:
            current_dist_to_goal = np.linalg.norm(robot_pose[0:2] - goal_pose)
            self.initial_distance = current_dist_to_goal
        
        dist_traveled = self.initial_distance - dist_to_goal
        
        # Compute desired velocity based on phase
        desired_speed = np.zeros_like(dist_to_goal)
        
        for i in range(len(dist_to_goal)):
            # Phase 1: Acceleration
            if dist_traveled[i] < self.accel_distance:
                # Linear ramp up
                desired_speed[i] = (dist_traveled[i] / self.accel_distance) * max_v
                
            # Phase 3: Deceleration (close to goal)
            elif dist_to_goal[i] < self.decel_distance:
                # Linear ramp down
                desired_speed[i] = (dist_to_goal[i] / self.decel_distance) * max_v
                
            # Phase 2: Cruise (in between)
            else:
                desired_speed[i] = max_v
        
        # Ensure minimum velocity
        desired_speed = np.maximum(desired_speed, 0.05)
        
        # Score: minimize difference between actual and desired velocity
        score = 1.0 - np.abs(vel - desired_speed) / max_v
        score = np.clip(score, 0.0, 1.0)
        
        return score
    
    def reset_profile(self):
        """Reset the initial distance for new goal"""
        self.initial_distance = None


class task_2(Node):
    """
    Enhanced DWA controller with trapezoidal velocity profile.
    Implements smooth acceleration and deceleration.
    """
    
    def __init__(self):
        super().__init__('task_2')
        # Parameters
        self.declare_parameter('alpha', 0.35)   # heading weight (slightly increased)
        self.declare_parameter('beta', 5.0)     # velocity weight (increased for speed preference)
        self.declare_parameter('gamma', 0.4)    # obstacle weight
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('collision_radius', 0.20)
        self.declare_parameter('collision_tolerance', 0.18)
        self.declare_parameter('num_ranges', 18)
        self.declare_parameter('max_lidar_range', 3.5)
        self.declare_parameter('feedback_steps', 50)
        
        # Trapezoidal velocity profile parameters
        self.declare_parameter('accel_distance', 0.3)   # distance to reach max speed
        self.declare_parameter('decel_distance', 0.4)   # distance to start slowing down
        self.declare_parameter('min_cruise_distance', 0.2)  # minimum cruise phase
        
        # Velocity limits
        self.declare_parameter('max_linear_vel', 0.35)   # Increased for faster navigation
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('max_linear_acc', 3.5)
        self.declare_parameter('max_angular_acc', 4.5)
        
        # State
        self.goal_pose = None
        self.current_pose = None  
        self.laser_ranges = None
        self.laser_angles = None
        self.last_cmd = np.array([0.0, 0.0])
        self.control_step = 0
        self.task_start_time = time.time()
        self.max_control_steps = 1000  

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/dwa_feedback', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        # Comment out dynamic goal to use static goal only
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback_odom, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback_ps, 10)

        # Timer for main control loop
        timer_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(timer_period, self.control_callback)

        # Enhanced DWA Planner with trapezoidal profile
        self.dwa = DWA_Enhanced(
            dt=1.0/self.get_parameter('control_rate').value,
            weight_angle=self.get_parameter('alpha').value,
            weight_vel=self.get_parameter('beta').value,
            weight_obs=self.get_parameter('gamma').value,
            accel_distance=self.get_parameter('accel_distance').value,
            decel_distance=self.get_parameter('decel_distance').value,
            min_cruise_distance=self.get_parameter('min_cruise_distance').value,
            radius=self.get_parameter('collision_radius').value,
            collision_tol=self.get_parameter('collision_tolerance').value,
            v_samples=10,  
            w_samples=20,
            max_lin_vel=self.get_parameter('max_linear_vel').value,
            max_ang_vel=self.get_parameter('max_angular_vel').value,
            max_linear_acc=self.get_parameter('max_linear_acc').value,
            max_ang_acc=self.get_parameter('max_angular_acc').value,
            init_pose=[0, 0, 0],  
        )

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        finite = np.isfinite(ranges)
        if not np.any(finite):
            return
        min_val = np.nanmin(ranges[finite])
        max_val = np.nanmax(ranges[finite])
        ranges = np.where(np.isnan(ranges), min_val, ranges)
        ranges = np.where(np.isinf(ranges), max_val, ranges)
        ranges = np.clip(ranges, 0.0, self.get_parameter('max_lidar_range').value)
        num_ranges = min(int(self.get_parameter('num_ranges').value), len(ranges))

        beam_angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        filtered_ranges = []
        filtered_angles = []
        range_sectors = np.array_split(ranges, num_ranges)
        angle_sectors = np.array_split(beam_angles, num_ranges)
        for r_sector, a_sector in zip(range_sectors, angle_sectors):
            if len(r_sector) == 0:
                filtered_ranges.append(self.get_parameter('max_lidar_range').value)
                filtered_angles.append(0.0)
                continue
            min_idx = int(np.argmin(r_sector))
            filtered_ranges.append(r_sector[min_idx])
            filtered_angles.append(a_sector[min_idx])

        self.laser_ranges = np.array(filtered_ranges)
        self.laser_angles = np.array(filtered_angles)
        self.get_logger().debug("Received LaserScan.")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = np.array([pos.x, pos.y, yaw])

    def goal_callback_odom(self, msg):
        self.goal_pose = msg.pose.pose
        self.control_step = 0
        self.task_start_time = time.time()
        self.dwa.reset_profile()  # Reset velocity profile for new goal
        self.get_logger().info(f"Received new goal (Odometry) at ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def goal_callback_ps(self, msg):
        self.goal_pose = msg.pose
        self.control_step = 0
        self.task_start_time = time.time()
        self.dwa.reset_profile()  # Reset velocity profile for new goal
        self.get_logger().info(f"Received new goal (PoseStamped) at ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def control_callback(self):
        if self.goal_pose is None or self.laser_ranges is None or self.laser_angles is None or self.current_pose is None:
            return

        # Safety: stop if too close to obstacle
        if np.min(self.laser_ranges) < self.get_parameter('collision_tolerance').value:
            self.stop_robot()
            self.publish_event('Collision')
            return

        # Convert goal_pose to [x, y]
        goal_xy = np.array([self.goal_pose.position.x, self.goal_pose.position.y])

        # Convert scan to obstacle coordinates
        obstacles = self.scan_to_obstacles(self.current_pose, self.laser_ranges, self.laser_angles)

        # Sync internal DWA robot state
        self.dwa.robot.pose = self.current_pose.copy()
        self.dwa.robot.vel = self.last_cmd.copy()

        # DWA: compute control
        v, w = self.dwa.compute_cmd(goal_xy, self.current_pose, obstacles)
        
        # Clamp velocities to safe limits
        max_v = self.get_parameter('max_linear_vel').value
        max_w = self.get_parameter('max_angular_vel').value
        v = np.clip(v, -max_v, max_v)
        w = np.clip(w, -max_w, max_w)
        
        # Calculate distance to goal for logging
        dist_to_goal = self.compute_distance(self.current_pose, self.goal_pose)
        
        # Determine current phase
        if self.dwa.initial_distance is not None:
            dist_traveled = self.dwa.initial_distance - dist_to_goal
            if dist_traveled < self.get_parameter('accel_distance').value:
                phase = "ACCEL"
            elif dist_to_goal < self.get_parameter('decel_distance').value:
                phase = "DECEL"
            else:
                phase = "CRUISE"
        else:
            phase = "INIT"
        
        self.get_logger().info(f"[{phase}] v={v:.2f}, w={w:.2f}, dist={dist_to_goal:.2f}")

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        self.last_cmd = np.array([v, w])

        # Check for goal reached
        if dist_to_goal < 0.15:
            self.stop_robot()
            self.publish_event('Goal Reached')
            elapsed_time = time.time() - self.task_start_time
            self.get_logger().info(f"Goal reached in {elapsed_time:.2f} seconds!")
            return

        # Timeout
        self.control_step += 1
        if self.control_step > self.max_control_steps:
            self.stop_robot()
            self.publish_event('Timeout')
            return

        # Feedback every N steps
        if self.control_step % self.get_parameter('feedback_steps').value == 0:
            self.publish_feedback(dist_to_goal)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_cmd = np.array([0.0, 0.0])

    def publish_event(self, event):
        msg = String()
        msg.data = f"Event: {event}"
        self.feedback_pub.publish(msg)
        self.get_logger().info(msg.data)

    def publish_feedback(self, dist):
        msg = String()
        msg.data = f"Distance to goal: {dist:.2f} m"
        self.feedback_pub.publish(msg)

    def compute_distance(self, pose1, pose2):
        dx = pose1[0] - pose2.position.x
        dy = pose1[1] - pose2.position.y
        return math.hypot(dx, dy)

    def scan_to_obstacles(self, robot_pose, scan_ranges, scan_angles):
        obs = []
        for r, a in zip(scan_ranges, scan_angles):
            if r < self.get_parameter('max_lidar_range').value:
                x = robot_pose[0] + r * math.cos(robot_pose[2] + a)
                y = robot_pose[1] + r * math.sin(robot_pose[2] + a)
                obs.append([x, y])
        return np.array(obs)

def main(args=None):
    rclpy.init(args=args)
    node = task_2()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
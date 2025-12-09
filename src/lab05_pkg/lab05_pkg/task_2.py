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
    Enhanced DWA with modified objective function:
    J = α·heading + β·vel' + γ·dist_obst + δ·dist_target
    
    where:
    - vel' is a decreasing term near goal (slow down)
    - dist_target keeps target in sight at desired distance
    """
    
    def __init__(self, slow_down_distance=1.0, target_distance=1.5, weight_target=0.15, **kwargs):
        super().__init__(**kwargs)
        self.slow_down_distance = slow_down_distance
        self.target_distance = target_distance
        self.weight_target = weight_target
        self.target_pose = None
        
    def set_target(self, target_pose):
        """Set the target pose for dynamic following task"""
        self.target_pose = target_pose
        
    def evaluate_paths(self, paths, velocities, goal_pose, robot_pose, obstacles):
        """
        Enhanced evaluation with new objective function:
        J = α·heading + β·vel' + γ·obst_dist + δ·dist_target
        """
        # detect nearest obstacle
        nearest_obs = calc_nearest_obs(robot_pose, obstacles)

        # Compute the scores for the generated path
        # (1) heading_angle and goal distance
        score_heading_angles = self.score_heading_angle(paths, goal_pose)
        # (2) velocity with slow-down near goal
        score_vel = self.score_vel_slowdown(velocities, paths, goal_pose)
        # (3) obstacles
        score_obstacles = self.score_obstacles(paths, nearest_obs)
        # (4) target tracking (if target is set)
        if self.target_pose is not None:
            score_target = self.score_target_distance(paths, self.target_pose)
        else:
            score_target = np.ones_like(score_vel)  # no penalty if no target

        # Scores Normalization
        score_heading_angles = normalize(score_heading_angles)
        score_vel = normalize(score_vel)
        score_obstacles = normalize(score_obstacles)
        score_target = normalize(score_target)

        # Compute the idx of the optimal path according to the overall score
        weights = np.array([[self.weight_angle, self.weight_vel, self.weight_obs, self.weight_target]]).T
        scores = np.array([score_heading_angles, score_vel, score_obstacles, score_target])
        
        opt_idx = np.argmax(np.sum(scores * weights, axis=0))

        try:
            return opt_idx
        except:
            raise Exception("Not possible to find an optimal path")
    
    def score_vel_slowdown(self, u, path, goal_pose):
        """
        Velocity score with slow-down behavior near goal.
        The robot should slow down when approaching the goal.
        
        Strategy: Scale velocity based on distance to goal
        - Far from goal: full velocity preference
        - Near goal: velocity is scaled down linearly with distance
        """
        vel = u[:, 0]
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)

        max_v = getattr(self.robot, "max_lin_vel", 1.0)
        s_dist = max(self.slow_down_distance, 1e-6)

        desired_speed = np.where(
            dist_to_goal < s_dist,
            (dist_to_goal / s_dist) * max_v,
            max_v
        )

        score = 1.0 - np.abs(vel - desired_speed) / max_v
        score = np.clip(score, 0.0, 1.0)

        return score


    def score_target_distance(self, path, target_pose):
        """
        Score trajectory based on distance to target.
        The robot should maintain a desired distance to the target for visibility.
        Penalizes both getting too close and too far from the target.
        """
        # Distance from each trajectory point to target
        target_dist = np.linalg.norm(path[:, -1, 0:2] - target_pose[0:2], axis=-1)
        
        # Desired distance to target
        desired_dist = self.target_distance
        
        # Score: penalize deviation from desired distance
        # Uses squared error from desired distance
        dist_error = np.abs(target_dist - desired_dist)
        
        # High score when distance is close to desired, low score when far away
        score = np.exp(-dist_error ** 2 / (desired_dist ** 2))
        
        return score


class task_2(Node):
    """
    Enhanced DWA controller for dynamic robot following task.
    Implements modified objective function with velocity slow-down and target tracking.
    """
    
    def __init__(self):
        super().__init__('task_2')
        # Parameters
        self.declare_parameter('alpha', 0.12)  # heading weight
        self.declare_parameter('beta', 0.8)    # velocity weight (reduced)
        self.declare_parameter('gamma', 0.4)   # obstacle weight
        self.declare_parameter('delta', 0.2)   # target distance weight
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('collision_radius', 0.20)  # meters
        self.declare_parameter('collision_tolerance', 0.18)  # meters
        self.declare_parameter('num_ranges', 18)
        self.declare_parameter('max_lidar_range', 3.5)
        self.declare_parameter('feedback_steps', 50)
        self.declare_parameter('slow_down_distance', 1.0)  # distance to start slowing down
        self.declare_parameter('target_distance', 1.5)     # desired distance to target
        
        # State
        self.goal_pose = None
        self.target_pose = None  # For dynamic following task
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
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback_odom, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback_ps, 10)
        self.create_subscription(Odometry, '/target_pose', self.target_callback, 10)

        # Timer for main control loop
        timer_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(timer_period, self.control_callback)

        # Enhanced DWA Planner
        self.dwa = DWA_Enhanced(
            dt=1.0/self.get_parameter('control_rate').value,
            weight_angle=self.get_parameter('alpha').value,
            weight_vel=self.get_parameter('beta').value,
            weight_obs=self.get_parameter('gamma').value,
            weight_target=self.get_parameter('delta').value,
            slow_down_distance=self.get_parameter('slow_down_distance').value,
            target_distance=self.get_parameter('target_distance').value,
            radius=self.get_parameter('collision_radius').value,
            collision_tol=self.get_parameter('collision_tolerance').value,
            v_samples=10,  
            w_samples=20, 
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

        # Pre-compute beam angles from the real scan metadata
        beam_angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Downsample while preserving all beams (handles remainder beams as well)
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
        self.get_logger().info(f"Min laser range: {np.min(self.laser_ranges):.2f}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = np.array([pos.x, pos.y, yaw])
        self.get_logger().debug(f"Odometry: {self.current_pose}")

    def goal_callback_odom(self, msg):
        self.goal_pose = msg.pose.pose
        self.control_step = 0
        self.task_start_time = time.time()
        self.get_logger().info(f"Received new goal (Odometry) at ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def goal_callback_ps(self, msg):
        self.goal_pose = msg.pose
        self.control_step = 0
        self.task_start_time = time.time()
        self.get_logger().info(f"Received new goal (PoseStamped) at ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def target_callback(self, msg):
        """Receive target pose for dynamic following task"""
        target_pose = msg.pose.pose
        target_array = np.array([target_pose.position.x, target_pose.position.y])
        self.target_pose = target_array
        self.dwa.set_target(target_array)
        self.get_logger().info(f"Received target at ({target_pose.position.x:.2f}, {target_pose.position.y:.2f})")

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

        # Sync internal DWA robot state with the latest odometry and last command
        self.dwa.robot.pose = self.current_pose.copy()
        self.dwa.robot.vel = self.last_cmd.copy()

        # DWA: compute control
        v, w = self.dwa.compute_cmd(goal_xy, self.current_pose, obstacles)
        self.get_logger().info(f"DWA output: v={v:.2f}, w={w:.2f}")

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        self.last_cmd = np.array([v, w])

        # Check for goal reached
        dist_to_goal = self.compute_distance(self.current_pose, self.goal_pose)
        if dist_to_goal < 0.15:
            self.stop_robot()
            self.publish_event('Goal')
            return

        # Timeout
        self.control_step += 1
        if self.control_step > self.max_control_steps:
            self.stop_robot()
            self.publish_event('Timeout')
            return

        # Feedback every N steps
        if self.control_step % self.get_parameter('feedback_steps').value == 0:
            dx = goal_xy[0] - self.current_pose[0]
            dy = goal_xy[1] - self.current_pose[1]
            angle_to_goal = math.atan2(dy, dx) - self.current_pose[2]
            self.get_logger().info(f"Distance to goal: {math.hypot(dx, dy):.2f}, Angle to goal: {math.degrees(angle_to_goal):.2f}")
            
            # Additional feedback for target tracking
            if self.target_pose is not None:
                dist_to_target = np.linalg.norm(self.current_pose[0:2] - self.target_pose)
                self.get_logger().info(f"Distance to target: {dist_to_target:.2f}")
            
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
        self.get_logger().info(msg.data)

    def compute_distance(self, pose1, pose2):
        # pose1: np.array([x, y, theta])
        # pose2: geometry_msgs.msg.Pose
        dx = pose1[0] - pose2.position.x
        dy = pose1[1] - pose2.position.y
        return math.hypot(dx, dy)

    def scan_to_obstacles(self, robot_pose, scan_ranges, scan_angles):
        # robot_pose: [x, y, theta]
        # scan_ranges/scan_angles: downsampled beams with matching indices
        obs = []
        for r, a in zip(scan_ranges, scan_angles):
            if r < self.get_parameter('max_lidar_range').value:
                # Transform to global frame
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
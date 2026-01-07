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

class task_1(Node):
    def __init__(self):
        super().__init__('task_1')
        # Parameters
        self.declare_parameter('alpha', 0.12)
        self.declare_parameter('beta', 1.0)
        self.declare_parameter('gamma', 0.4)
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('collision_radius', 0.20)  
        self.declare_parameter('collision_tolerance', 0.18)
        self.declare_parameter('num_ranges', 18)
        self.declare_parameter('max_lidar_range', 3.5)
        self.declare_parameter('feedback_steps', 50)
        self.declare_parameter('max_linear_vel', 0.21)
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('tracking_threshold', 2.0)
        self.declare_parameter('max_bearing_error', 90.0) 
        
        # State
        self.goal_pose = None
        self.current_pose = None  
        self.laser_ranges = None
        self.laser_angles = None
        self.last_cmd = np.array([0.0, 0.0])
        self.control_step = 50
        self.task_start_time = None
        self.max_control_steps = 100000
        self.collision_detected = False
        self.goals_reached = 0
        self.failures = {'Timeout': 0, 'Collision': 0,}
        self.prev_distance_to_target = None  # Track if getting closer/farther

        # Metrics tracking
        self.target_ground_truth = None  # Target position and computed heading
        self.prev_target_ground_truth = None
        self.tracking_time = 0.0
        self.total_time = 0.0
        self.distance_errors = []
        self.bearing_errors = []
        self.obstacle_distances = []
        self.last_metrics_time = time.time()

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/dwa_feedback', 10)
        self.metrics_pub = self.create_publisher(String, '/metrics', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback_odom, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback_ps, 10)
        self.create_subscription(Odometry, '/ground_truth', self.ground_truth_callback, 10)

        # Timer 
        timer_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(timer_period, self.control_callback)
        self.metrics_timer = self.create_timer(5.0, self.publish_metrics) # every 5 seconds

        # DWA Planner
        self.dwa = DWA(
            dt=1.0/self.get_parameter('control_rate').value,
            weight_angle=self.get_parameter('alpha').value,
            weight_vel=self.get_parameter('beta').value,
            weight_obs=self.get_parameter('gamma').value,
            radius=self.get_parameter('collision_radius').value,
            collision_tol=self.get_parameter('collision_tolerance').value,
            v_samples=10,  
            w_samples=20,
            max_lin_vel=self.get_parameter('max_linear_vel').value,
            init_pose=[0, 0, 0],)

    def ground_truth_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.prev_target_ground_truth = self.target_ground_truth
        self.target_ground_truth = np.array([pos.x, pos.y, yaw])
        
        self.get_logger().debug(f"Target ground truth: pos=({pos.x:.2f},{pos.y:.2f}), yaw={math.degrees(yaw):.1f}°")

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

        beam_angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment # angle for each beam

        # Downsample 
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
        
        # Store obstacle distances for metrics
        valid_ranges = self.laser_ranges[self.laser_ranges < self.get_parameter('max_lidar_range').value]
        if len(valid_ranges) > 0:
            self.obstacle_distances.extend(valid_ranges.tolist())
        
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
        pos = msg.pose.pose.position    # Use the goal pose as target position
        
        # Calculate target's heading from velocity (change in position)
        if self.prev_target_ground_truth is not None and self.target_ground_truth is not None:
            dx = pos.x - self.target_ground_truth[0]
            dy = pos.y - self.target_ground_truth[1]
            
            if math.hypot(dx, dy) > 0.01:                   # If target moved, use velocity direction as heading
                target_yaw = math.atan2(dy, dx)
            else:
                target_yaw = self.target_ground_truth[2]    # Keep previous heading if stationary
        else:
            target_yaw = 0.0 # Default heading if no previous data
        
        self.prev_target_ground_truth = self.target_ground_truth
        self.target_ground_truth = np.array([pos.x, pos.y, target_yaw])
        
        if self.task_start_time is None:
            self.task_start_time = time.time()
        self.get_logger().info(f"Received new goal at ({pos.x:.2f}, {pos.y:.2f}), heading={math.degrees(target_yaw):.1f}°")

    def goal_callback_ps(self, msg):
        self.goal_pose = msg.pose

        if self.task_start_time is None:         # Only reset start time if this is the first goal
            self.task_start_time = time.time()
        self.get_logger().info(f"Received new goal (PoseStamped) at ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def compute_tracking_metrics(self): 
        if self.target_ground_truth is None or self.current_pose is None:
            return False, "NO_DATA"
        
        current_time = time.time()
        dt = current_time - self.last_metrics_time
        self.last_metrics_time = current_time
        self.total_time += dt
        
        # Compute actual distance to target
        dx = self.target_ground_truth[0] - self.current_pose[0]
        dy = self.target_ground_truth[1] - self.current_pose[1]
        actual_distance = math.hypot(dx, dy)
        
        # Bearing: difference between robot heading and target's heading
        bearing_angle = self.target_ground_truth[2] - self.current_pose[2]
        bearing_angle = math.atan2(math.sin(bearing_angle), math.cos(bearing_angle))
        
        # Check if distance is increasing (moving away)
        distance_increasing = False
        if self.prev_distance_to_target is not None:
            distance_increasing = actual_distance > self.prev_distance_to_target + 0.5  # 5cm tolerance
        self.prev_distance_to_target = actual_distance
        
        # Tracking 
        tracking_threshold = self.get_parameter('tracking_threshold').value
        max_bearing_error_deg = self.get_parameter('max_bearing_error').value
        max_bearing_error_rad = math.radians(max_bearing_error_deg)
        
        # TRACKING if: 1. Within distance range, 2. Heading aligned with target (bearing not too large) 3. NOT moving away from target
        is_tracking = (
            actual_distance <= tracking_threshold and
            abs(bearing_angle) <= max_bearing_error_rad and
            not distance_increasing)
        
        # reason for losing track
        lost_reason = "OK"
        if actual_distance > tracking_threshold:
            lost_reason = "TOO_FAR"
        elif abs(bearing_angle) > max_bearing_error_rad:
            lost_reason = f"WRONG_DIRECTION({math.degrees(bearing_angle):.0f}°)"
        elif distance_increasing:
            lost_reason = "MOVING_AWAY"
        
        if is_tracking:
            self.tracking_time += dt            
            desired_distance = self.get_parameter('desired_distance').value
            distance_error = actual_distance - desired_distance
            self.distance_errors.append(distance_error)
            self.bearing_errors.append(bearing_angle)
        
        self.get_logger().debug(
            f"Distance: {actual_distance:.2f}m, Bearing: {math.degrees(bearing_angle):.1f}°, "
            f"Dist_change: {'+' if distance_increasing else '-'}, Tracking: {is_tracking}, Reason: {lost_reason}")
    
        return is_tracking, lost_reason

    def check_goal_completion(self): #Check if the previous goal was successfully reached
        if self.goal_pose is None or self.current_pose is None:
            return
        
        goal_xy = np.array([self.goal_pose.position.x, self.goal_pose.position.y])
        dist_to_goal = math.hypot(goal_xy[0] - self.current_pose[0], goal_xy[1] - self.current_pose[1])
        
        goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        
        if dist_to_goal <= goal_reached_threshold:
            self.goals_reached += 1
            self.get_logger().info(f"✓ Goal reached! (distance: {dist_to_goal:.2f}m)")
        else:
            self.get_logger().warn(f"✗ Goal not reached (distance: {dist_to_goal:.2f}m)")

    def control_callback(self):
        if self.goal_pose is None or self.laser_ranges is None or self.laser_angles is None or self.current_pose is None:
            return

        is_tracking, lost_reason = self.compute_tracking_metrics()   # Compute tracking metrics

        if np.min(self.laser_ranges) < self.get_parameter('collision_tolerance').value:   # stop if too close to obstacle (only count collision once)
            if not self.collision_detected:
                self.failures['Collision'] += 1
                self.collision_detected = True
                self.publish_event('Collision')
            self.stop_robot()
            return

        goal_xy = np.array([self.goal_pose.position.x, self.goal_pose.position.y])  # Convert goal_pose to [x, y]
        obstacles = self.scan_to_obstacles(self.current_pose, self.laser_ranges, self.laser_angles) # Convert scan to obstacle coordinates

        self.dwa.robot.pose = self.current_pose.copy()
        self.dwa.robot.vel = self.last_cmd.copy()

        v, w = self.dwa.compute_cmd(goal_xy, self.current_pose, obstacles) # DWA: compute control
        
        # Clamp velocities
        max_v = self.get_parameter('max_linear_vel').value
        max_w = self.get_parameter('max_angular_vel').value
        v = np.clip(v, -max_v, max_v)
        w = np.clip(w, -max_w, max_w)
        
        # Calculate distances
        dist_to_goal = math.hypot(goal_xy[0] - self.current_pose[0], goal_xy[1] - self.current_pose[1])
        
        if self.target_ground_truth is not None:
            dx_gt = self.target_ground_truth[0] - self.current_pose[0]
            dy_gt = self.target_ground_truth[1] - self.current_pose[1]
            dist_to_gt = math.hypot(dx_gt, dy_gt)
            
            bearing_angle = self.target_ground_truth[2] - self.current_pose[2]
            bearing_angle = math.atan2(math.sin(bearing_angle), math.cos(bearing_angle))
            
            tracking_status = "TRACKING" if is_tracking else f"LOST({lost_reason})"
            
            self.get_logger().info(
                f"[{tracking_status}] v={v:.2f}, w={w:.2f} | "
                f"dist_target={dist_to_gt:.2f}m, bearing={math.degrees(bearing_angle):.1f}°")
        else:
            self.get_logger().info(f"v={v:.2f}, w={w:.2f} | dist_goal={dist_to_goal:.2f}m (no ground truth)")

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        self.last_cmd = np.array([v, w])

        self.control_step += 1

        # Timeout check
        if self.max_control_steps is not None and self.control_step > self.max_control_steps:
            self.stop_robot()
            self.publish_event('Timeout')
            self.failures['Timeout'] += 1
            return

        # Feedback every N steps
        if self.control_step % self.get_parameter('feedback_steps').value == 0:
            dx = goal_xy[0] - self.current_pose[0]
            dy = goal_xy[1] - self.current_pose[1]
            angle_to_goal = math.atan2(dy, dx) - self.current_pose[2]
            self.get_logger().info(f"Distance to goal: {dist_to_goal:.2f}, Angle to goal: {math.degrees(angle_to_goal):.2f}")
            self.publish_feedback(dist_to_goal)

    def publish_metrics(self):
        if self.total_time == 0:
            return
        
        tracking_percentage = (self.tracking_time / self.total_time) * 100.0 # Tracking time percentage
        
        # RMSE for distance and bearing
        distance_rmse = 0.0
        bearing_rmse_deg = 0.0
        avg_bearing = 0.0
        max_bearing = 0.0
        
        if len(self.distance_errors) > 0:
            distance_rmse = math.sqrt(np.mean(np.array(self.distance_errors)**2))
        
        if len(self.bearing_errors) > 0:
            bearing_errors_array = np.array(self.bearing_errors)
            
            sin_errors = np.sin(bearing_errors_array)
            cos_errors = np.cos(bearing_errors_array)
            mean_sin = np.mean(sin_errors)
            mean_cos = np.mean(cos_errors)
            
            mean_angle = math.atan2(mean_sin, mean_cos)
            
            angular_diffs = bearing_errors_array - mean_angle
            angular_diffs = np.arctan2(np.sin(angular_diffs), np.cos(angular_diffs))
            
            bearing_rmse = math.sqrt(np.mean(angular_diffs**2))
            bearing_rmse_deg = math.degrees(bearing_rmse)
            
            avg_bearing = math.degrees(mean_angle)
            max_bearing = math.degrees(np.max(np.abs(angular_diffs)))
        
        # Obstacle distances
        avg_obstacle_dist = 0.0
        min_obstacle_dist = 0.0
        if len(self.obstacle_distances) > 0:
            avg_obstacle_dist = np.mean(self.obstacle_distances)
            min_obstacle_dist = np.min(self.obstacle_distances)
        
        # Create metrics message (compact format)
        metrics_msg = String()
        metrics_msg.data = (
            f"Track:{tracking_percentage:.1f}% T:{self.total_time:.1f}s "
            f"DistRMSE:{distance_rmse:.3f}m BearRMSE:{bearing_rmse_deg:.1f}° "
            f"ObstAvg:{avg_obstacle_dist:.2f}m Min:{min_obstacle_dist:.2f}m ")
        
        self.metrics_pub.publish(metrics_msg)
        self.get_logger().info(
            f"\n=== METRICS ===\n"
            f"Track Time: {tracking_percentage:.2f}% over {self.total_time:.2f}s\n"
            f"Distance RMSE: {distance_rmse:.3f}m\n"
            f"Bearing RMSE: {bearing_rmse_deg:.2f}°\n"
            f"Avg Obstacle Dist: {avg_obstacle_dist:.3f}m\n"
            f"Min Obstacle Dist: {min_obstacle_dist:.3f}m\n"
            f"Bearing Stats: avg={avg_bearing:.1f}°, max_dev={max_bearing:.1f}°, samples={len(self.bearing_errors)}")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_cmd = np.array([0.0, 0.0])

    def publish_event(self, event):
        msg = String()
        msg.data = f"Event: {event}"
        self.feedback_pub.publish(msg)
        self.get_logger().info(msg.data)
        
        # Publish final metrics on stop
        self.publish_metrics()

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
    node = task_1()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
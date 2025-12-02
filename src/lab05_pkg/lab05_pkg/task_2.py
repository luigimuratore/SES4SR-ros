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
sys.path.append('/home/ubuntu/Documents/SES4SR-ros/src/planning_control_methods/Controllers/DWA')
from dwa import DWA  
from utils import normalize, calc_nearest_obs


class task_2(Node):
    def __init__(self):
        super().__init__('task_2')
        # Parameters
        self.declare_parameter('alpha', 0.12) # heading weight
        self.declare_parameter('beta', 0.4)  # speed weight (for slowed term)
        self.declare_parameter('gamma', 0.8) # obstacle weight
        self.declare_parameter('delta', 0.15) # target-follow weight
        self.declare_parameter('slowdown_dist', 2.0) # start slowing near goal
        self.declare_parameter('follow_desired', 1.0) # preferred follow distance
        self.declare_parameter('follow_band', 0.5) # tolerance for follow distance
        self.declare_parameter('max_cmd_lin', 0.4) # clamp outgoing linear cmd
        self.declare_parameter('max_cmd_ang', 0.6) # clamp outgoing angular cmd
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('collision_radius', 0.20)  # meters
        self.declare_parameter('collision_tolerance', 0.18)  # meters
        self.declare_parameter('num_ranges', 18)
        self.declare_parameter('max_lidar_range', 3.5)
        self.declare_parameter('feedback_steps', 50)
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
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback_odom, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback_ps, 10)

        # Timer for main control loop
        timer_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(timer_period, self.control_callback)

        # DWA Planner (base class, we will apply custom objective in this node)
        self.dwa = DWA(
            dt=1.0/self.get_parameter('control_rate').value,
            weight_angle=self.get_parameter('alpha').value,
            weight_vel=self.get_parameter('beta').value,
            weight_obs=self.get_parameter('gamma').value,
            radius=self.get_parameter('collision_radius').value,
            collision_tol=self.get_parameter('collision_tolerance').value,
            v_samples=10,  
            w_samples=7, 
            max_lin_vel=0.4,
            max_ang_vel=0.8,
            max_linear_acc=0.4,
            max_ang_acc=0.6,
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

        # DWA: compute control with modified objective (slow near goal + target visibility)
        v, w = self.compute_cmd_custom(goal_xy, obstacles)
        # Clamp outgoing command to reduce oscillations
        max_lin = self.get_parameter('max_cmd_lin').value
        max_ang = self.get_parameter('max_cmd_ang').value
        v = float(np.clip(v, -max_lin, max_lin))
        w = float(np.clip(w, -max_ang, max_ang))
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

    def compute_cmd_custom(self, goal_pose, obstacles):
        # Sample trajectories from base DWA
        paths, velocities = self.dwa.get_trajectories(self.current_pose)

        # Scores
        heading = self.dwa.score_heading_angle(paths, goal_pose)
        vel_slow = self.score_vel_slowdown(velocities, paths, goal_pose)
        nearest_obs = calc_nearest_obs(self.current_pose, obstacles)
        obst_raw = self.dwa.score_obstacles(paths, nearest_obs)
        coll_mask = obst_raw < 0  # collisions scored as large negatives in DWA
        follow = self.score_follow_target(paths, goal_pose)

        # Normalize
        heading = normalize(heading)
        vel_slow = normalize(vel_slow)
        obst = normalize(obst_raw)
        follow = normalize(follow)

        total = (
            heading * self.get_parameter('alpha').value
            + vel_slow * self.get_parameter('beta').value
            + obst * self.get_parameter('gamma').value
            + follow * self.get_parameter('delta').value
        )
        total[coll_mask] = -np.inf

        if np.all(np.isneginf(total)):
            opt_idx = int(np.argmin(np.hypot(velocities[:, 0], velocities[:, 1])))
        else:
            opt_idx = int(np.argmax(total))

        return velocities[opt_idx]

    def score_vel_slowdown(self, u, path, goal_pose):
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)
        slowdown_dist = self.get_parameter('slowdown_dist').value
        factor = np.minimum(dist_to_goal / slowdown_dist, 1.0)
        return u[:, 0] * factor

    def score_follow_target(self, path, goal_pose):
        end_xy = path[:, -1, 0:2]
        end_th = path[:, -1, 2]
        dist = np.linalg.norm(end_xy - goal_pose, axis=-1)

        desired = self.get_parameter('follow_desired').value
        band = self.get_parameter('follow_band').value
        dist_score = np.exp(-np.abs(dist - desired) / band)

        bearing = np.arctan2(goal_pose[1] - end_xy[:, 1], goal_pose[0] - end_xy[:, 0])
        bearing_err = np.arctan2(np.sin(bearing - end_th), np.cos(bearing - end_th))
        vis_score = 0.5 * (np.cos(bearing_err) + 1.0)  # [0,1]

        return dist_score * vis_score

def main(args=None):
    rclpy.init(args=args)
    node = task_2()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

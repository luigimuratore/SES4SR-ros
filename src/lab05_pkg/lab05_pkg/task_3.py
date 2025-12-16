import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from landmark_msgs.msg import LandmarkArray
import numpy as np
import math
import time

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
dwa_path = os.path.join(current_dir, '../../planning_control_methods/Controllers/DWA')
sys.path.append(dwa_path)
from dwa import DWA  

class task_3(Node):
    def __init__(self):
        super().__init__('task_3')
        # Parameters
        self.declare_parameter('alpha', 0.12) # heading weight
        self.declare_parameter('beta', 1.0)  # speed weight
        self.declare_parameter('gamma', 0.4) # obstacle weight
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
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Subscribe to AprilTag landmarks from RGB-D camera
        self.create_subscription(LandmarkArray, '/camera/landmarks', self.landmark_callback, 10)

        # Timer for main control loop
        timer_period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(timer_period, self.control_callback)

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

    def landmark_callback(self, msg):
        """
        Process AprilTag landmarks from RGB-D camera.
        Transform range and bearing measurements to X-Y coordinates in odom frame.
        """
        if len(msg.landmarks) == 0:
            self.get_logger().debug("No landmarks detected.")
            return
        
        if self.current_pose is None:
            self.get_logger().warn("Cannot process landmarks without current pose.")
            return
        
        # Use the first detected landmark (you can modify to use specific tag ID)
        landmark = msg.landmarks[0]
        
        # Extract range and bearing from the landmark message
        # Assuming the landmark pose is in the camera frame
        # Convert to robot base frame then to odom frame
        landmark_x = landmark.pose.position.x
        landmark_y = landmark.pose.position.y
        
        # Calculate range and bearing from camera coordinates
        range_to_landmark = math.hypot(landmark_x, landmark_y)
        bearing_to_landmark = math.atan2(landmark_y, landmark_x)
        
        # Transform to odom frame using current robot pose
        # Robot pose: [x, y, theta] in odom frame
        # Landmark in odom frame = robot_pose + R(theta) * [range*cos(bearing), range*sin(bearing)]
        goal_x = self.current_pose[0] + range_to_landmark * math.cos(self.current_pose[2] + bearing_to_landmark)
        goal_y = self.current_pose[1] + range_to_landmark * math.sin(self.current_pose[2] + bearing_to_landmark)
        
        # Create goal pose
        self.goal_pose = Pose()
        self.goal_pose.position.x = goal_x
        self.goal_pose.position.y = goal_y
        self.goal_pose.position.z = 0.0
        
        # Reset control state for new goal
        self.control_step = 0
        self.task_start_time = time.time()
        
        self.get_logger().info(
            f"Detected AprilTag (ID: {landmark.id}): "
            f"range={range_to_landmark:.2f}m, bearing={math.degrees(bearing_to_landmark):.1f}deg -> "
            f"Goal in odom: ({goal_x:.2f}, {goal_y:.2f})"
        )

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
    node = task_3()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

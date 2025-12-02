import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math
import time

# Import your DWA logic (adapt the import as needed)
import sys
sys.path.append('/home/ubuntu/Documents/SES4SR-ros/src/planning_control_methods/Controllers/DWA')
from dwa import DWA  # Correct import for your workspace

class DWAControllerNode(Node):
    def __init__(self):
        super().__init__('dwa_controller_node')
        # Parameters
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('beta', 0.2)
        self.declare_parameter('gamma', 0.3)
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('collision_radius', 0.20)  # meters
        self.declare_parameter('collision_tolerance', 0.18)  # meters
        self.declare_parameter('num_ranges', 18)
        self.declare_parameter('max_lidar_range', 3.5)
        self.declare_parameter('feedback_steps', 50)
        # State
        self.goal_pose = None
        self.current_pose = None  # You may need odometry for this
        self.laser_ranges = None
        self.control_step = 0
        self.task_start_time = time.time()
        self.max_control_steps = 1000  # Set as needed

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/dwa_feedback', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # TODO: Subscribe to odometry if needed for robot pose

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
            v_samples=10,  # or tune as needed
            w_samples=20,  # or tune as needed
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
        num_ranges = self.get_parameter('num_ranges').value
        sector_size = len(ranges) // num_ranges
        filtered = []
        for i in range(num_ranges):
            sector = ranges[i*sector_size:(i+1)*sector_size]
            filtered.append(np.min(sector) if len(sector) > 0 else self.get_parameter('max_lidar_range').value)
        self.laser_ranges = np.array(filtered)

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.control_step = 0
        self.task_start_time = time.time()
        self.get_logger().info("Received new goal!")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = np.array([pos.x, pos.y, yaw])

    def control_callback(self):
        if self.goal_pose is None or self.laser_ranges is None or self.current_pose is None:
            return

        # Safety: stop if too close to obstacle
        if np.min(self.laser_ranges) < self.get_parameter('collision_tolerance').value:
            self.stop_robot()
            self.publish_event('Collision')
            return

        # Convert goal_pose to [x, y]
        goal_xy = np.array([self.goal_pose.position.x, self.goal_pose.position.y])

        # Convert scan to obstacle coordinates
        obstacles = self.scan_to_obstacles(self.current_pose, self.laser_ranges)

        # DWA: compute control
        v, w = self.dwa.compute_cmd(goal_xy, self.current_pose, obstacles)

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

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
            self.publish_feedback(dist_to_goal)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

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
        # TODO: Implement based on your pose representation
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.hypot(dx, dy)

    def scan_to_obstacles(self, robot_pose, scan_ranges):
        # robot_pose: [x, y, theta]
        num_ranges = len(scan_ranges)
        angle_min = -math.pi / 2  # adjust if needed
        angle_max = math.pi / 2   # adjust if needed
        angles = np.linspace(angle_min, angle_max, num_ranges)
        obs = []
        for r, a in zip(scan_ranges, angles):
            if r < self.get_parameter('max_lidar_range').value:
                # Transform to global frame
                x = robot_pose[0] + r * math.cos(robot_pose[2] + a)
                y = robot_pose[1] + r * math.sin(robot_pose[2] + a)
                obs.append([x, y])
        return np.array(obs)

def main(args=None):
    rclpy.init(args=args)
    node = DWAControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
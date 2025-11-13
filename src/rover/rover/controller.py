import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import tf_transformations

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.2)
        self.declare_parameter('max_turn_rate', 1.0)
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('is_active', True)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.is_active = self.get_parameter('is_active').value
        
        self.get_logger().info(f'Parameters loaded: max_speed={self.max_speed}, max_turn_rate={self.max_turn_rate}, is_active={self.is_active}')

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Obstacle detection
        self.closest_range_front = float('inf')
        self.closest_range_left = float('inf')
        self.closest_range_right = float('inf')
        
        # Odometry tracking
        self.yaw = 0.0
        self.last_odom_time = None
        self.odom_received = False
        self.scan_received = False
        self.odom_timeout = 2.0  # Increased to 2 seconds
        
        # State machine for obstacle avoidance
        self.state = 'FORWARD'
        self.turn_target_yaw = None
        self.turn_tolerance = 0.15
        
        self.get_logger().info('Controller initialized and ready')
        self.get_logger().info('Waiting for odometry and scan data...')

    def odom_callback(self, msg):
        """Extract yaw angle from odometry"""
        if not self.odom_received:
            self.get_logger().info('First odometry message received!')
            self.odom_received = True
            
        self.last_odom_time = self.get_clock().now()
        
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)

    def scan_callback(self, msg):
        """Process LiDAR data to detect obstacles"""
        if not self.scan_received:
            self.get_logger().info(f'First scan message received! {len(msg.ranges)} points')
            self.scan_received = True
            
        if len(msg.ranges) == 0:
            return
        
        # Calculate angle increment
        num_readings = len(msg.ranges)
        
        # Define sectors
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            
            # Calculate angle for this reading
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)
            
            # Classify into sectors
            if -30 <= angle_deg <= 30:
                front_ranges.append(r)
            elif 30 < angle_deg <= 90:
                left_ranges.append(r)
            elif -90 <= angle_deg < -30:
                right_ranges.append(r)
        
        # Find minimum distances
        self.closest_range_front = min(front_ranges) if front_ranges else float('inf')
        self.closest_range_left = min(left_ranges) if left_ranges else float('inf')
        self.closest_range_right = min(right_ranges) if right_ranges else float('inf')

    def _determine_clearest_side(self):
        """Determine which side has more clearance"""
        if self.closest_range_left > self.closest_range_right:
            return 'LEFT'
        else:
            return 'RIGHT'

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _shortest_angular_dist(self, from_angle, to_angle):
        """Calculate shortest angular distance between two angles"""
        diff = to_angle - from_angle
        return self._normalize_angle(diff)

    def timer_callback(self):
        """Main control loop"""
        # Check if active
        if not self.is_active:
            self.get_logger().warn('Controller is INACTIVE. Set is_active parameter to true.', throttle_duration_sec=5.0)
            self._stop_robot()
            return
        
        # Check if we have received sensor data
        if not self.odom_received:
            self.get_logger().warn('No odometry data received yet. Waiting...', throttle_duration_sec=2.0)
            self._stop_robot()
            return
            
        if not self.scan_received:
            self.get_logger().warn('No LiDAR scan data received yet. Waiting...', throttle_duration_sec=2.0)
            self._stop_robot()
            return
        
        # Check if odometry is recent
        time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if time_since_odom > self.odom_timeout:
            self.get_logger().warn(f'Odometry timeout ({time_since_odom:.2f}s)', throttle_duration_sec=2.0)
            self._stop_robot()
            return
        
        # State machine for obstacle avoidance
        cmd = Twist()
        
        if self.state == 'FORWARD':
            # Check for obstacles in front
            if self.closest_range_front < self.obstacle_threshold:
                # Obstacle detected! Decide which way to turn
                turn_direction = self._determine_clearest_side()
                
                if turn_direction == 'LEFT':
                    self.state = 'TURN_LEFT'
                    self.turn_target_yaw = self._normalize_angle(self.yaw + math.pi/2)
                    self.get_logger().info(f'Obstacle at {self.closest_range_front:.2f}m! Turning LEFT to {math.degrees(self.turn_target_yaw):.1f}°')
                else:
                    self.state = 'TURN_RIGHT'
                    self.turn_target_yaw = self._normalize_angle(self.yaw - math.pi/2)
                    self.get_logger().info(f'Obstacle at {self.closest_range_front:.2f}m! Turning RIGHT to {math.degrees(self.turn_target_yaw):.1f}°')
            else:
                # No obstacle, move forward
                cmd.linear.x = -self.max_speed
                cmd.angular.z = 0.0
                self.get_logger().info(f'Moving FORWARD at {self.max_speed} m/s', throttle_duration_sec=2.0)
        
        elif self.state == 'TURN_LEFT':
            angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
            
            if abs(angle_diff) < self.turn_tolerance:
                self.get_logger().info(f'Turn complete. Current yaw: {math.degrees(self.yaw):.1f}°')
                self.state = 'FORWARD'
                self.turn_target_yaw = None
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_turn_rate
        
        elif self.state == 'TURN_RIGHT':
            angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
            
            if abs(angle_diff) < self.turn_tolerance:
                self.get_logger().info(f'Turn complete. Current yaw: {math.degrees(self.yaw):.1f}°')
                self.state = 'FORWARD'
                self.turn_target_yaw = None
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.max_turn_rate
        
        # Publish command
        self.publisher_.publish(cmd)
        
        # Debug info
        self.get_logger().info(
            f'State: {self.state}, Front: {self.closest_range_front:.2f}m, '
            f'L: {self.closest_range_left:.2f}m, R: {self.closest_range_right:.2f}m, '
            f'Yaw: {math.degrees(self.yaw):.1f}°',
            throttle_duration_sec=1.0
        )

    def _stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
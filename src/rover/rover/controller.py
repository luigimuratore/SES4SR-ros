import rclpy
import math
import signal
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('max_turn_rate', 0.8)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('is_active', True)
        self.declare_parameter('turn_tolerance', 0.15)  # radians

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.is_active = self.get_parameter('is_active').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odometry/fused', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Control timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Odometry state (from encoder only for position)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # From encoder
        self.linear_velocity = 0.0  # From encoder
        
        # IMU state (angular velocity)
        self.imu_angular_velocity = 0.0  # From IMU

        # LiDAR state
        self.closest_range_front = float('inf')
        self.closest_range_left = float('inf')
        self.closest_range_right = float('inf')

        # Sensor flags
        self.odom_received = False
        self.imu_received = False
        self.scan_received = False
        self.last_odom_time = None
        self.last_imu_time = None
        self.odom_timeout = 2.0

        # State machine
        self.state = 'FORWARD'
        self.turn_target_yaw = None

        self.get_logger().info('Controller initialized')

    def odom_callback(self, msg):
        """Extract fused odometry (encoder position + IMU yaw)"""
        if not self.odom_received:
            self.get_logger().info('Fused odometry received!')
            self.odom_received = True

        self.last_odom_time = self.get_clock().now()

        # Position from encoder
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Yaw from IMU (accurate!)
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)

        # Velocity
        self.linear_velocity = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        """Extract angular velocity from IMU"""
        if not self.imu_received:
            self.get_logger().info('IMU data received!')
            self.imu_received = True

        self.last_imu_time = self.get_clock().now()

        # Angular velocity from IMU (Z-axis for 2D rotation)
        self.imu_angular_velocity = msg.angular_velocity.z

    def scan_callback(self, msg):
        """Process LiDAR data"""
        if not self.scan_received:
            self.get_logger().info(f'LiDAR scan received! {len(msg.ranges)} points')
            self.scan_received = True

        if len(msg.ranges) == 0:
            return

        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            # Front: -30° to +30°
            if -30 <= angle_deg <= 30:
                front_ranges.append(r)
            # Left: +30° to +90°
            elif 30 < angle_deg <= 90:
                left_ranges.append(r)
            # Right: -90° to -30°
            elif -90 <= angle_deg < -30:
                right_ranges.append(r)

        self.closest_range_front = min(front_ranges) if front_ranges else float('inf')
        self.closest_range_left = min(left_ranges) if left_ranges else float('inf')
        self.closest_range_right = min(right_ranges) if right_ranges else float('inf')

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

    def _determine_clearest_side(self):
        """Determine which side has more clearance"""
        if self.closest_range_left > self.closest_range_right:
            return 'LEFT'
        else:
            return 'RIGHT'

    def _snap_to_cardinal_yaw(self, yaw):
        """Snap yaw to nearest cardinal direction (0, π/2, π, -π/2)"""
        cardinals = [0, math.pi / 2, math.pi, -math.pi / 2]
        return min(cardinals, key=lambda c: abs(self._shortest_angular_dist(yaw, c)))

    def timer_callback(self):
        """Main control loop"""
        # Check if active
        if not self.is_active:
            self.get_logger().warn('Controller INACTIVE', throttle_duration_sec=5.0)
            self._stop_robot()
            return

        # Check sensor data
        if not self.odom_received:
            self.get_logger().warn('Waiting for encoder odometry...', throttle_duration_sec=2.0)
            self._stop_robot()
            return

        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data...', throttle_duration_sec=2.0)
            self._stop_robot()
            return

        if not self.scan_received:
            self.get_logger().warn('Waiting for LiDAR scan...', throttle_duration_sec=2.0)
            self._stop_robot()
            return

        # Check odometry timeout
        time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if time_since_odom > self.odom_timeout:
            self.get_logger().warn(f'Encoder timeout ({time_since_odom:.2f}s)', throttle_duration_sec=2.0)
            self._stop_robot()
            return

        cmd = Twist()

        if self.state == 'FORWARD':
            if self.closest_range_front < self.obstacle_threshold:
                # Obstacle detected
                turn_direction = self._determine_clearest_side()

                if turn_direction == 'LEFT':
                    self.state = 'TURN_LEFT'
                    self.turn_target_yaw = self._normalize_angle(self.yaw + math.pi / 2)
                    self.get_logger().info(
                        f'Obstacle at {self.closest_range_front:.2f}m! Turning LEFT to {math.degrees(self.turn_target_yaw):.1f}°'
                    )
                else:
                    self.state = 'TURN_RIGHT'
                    self.turn_target_yaw = self._normalize_angle(self.yaw - math.pi / 2)
                    self.get_logger().info(
                        f'Obstacle at {self.closest_range_front:.2f}m! Turning RIGHT to {math.degrees(self.turn_target_yaw):.1f}°'
                    )
            else:
                # Move forward
                cmd.linear.x = self.max_speed
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'FORWARD | Front: {self.closest_range_front:.2f}m | Yaw: {math.degrees(self.yaw):.1f}°',
                    throttle_duration_sec=2.0
                )

        elif self.state == 'TURN_LEFT':
            angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)

            if abs(angle_diff) < self.turn_tolerance:
                self.get_logger().info(f'Turn LEFT complete. Yaw: {math.degrees(self.yaw):.1f}°')
                self.state = 'FORWARD'
                self.turn_target_yaw = None
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_turn_rate
                self.get_logger().info(
                    f'TURN_LEFT | Target: {math.degrees(self.turn_target_yaw):.1f}°, Current: {math.degrees(self.yaw):.1f}°, Diff: {math.degrees(angle_diff):.1f}°',
                    throttle_duration_sec=1.0
                )

        elif self.state == 'TURN_RIGHT':
            angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)

            if abs(angle_diff) < self.turn_tolerance:
                self.get_logger().info(f'Turn RIGHT complete. Yaw: {math.degrees(self.yaw):.1f}°')
                self.state = 'FORWARD'
                self.turn_target_yaw = None
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.max_turn_rate
                self.get_logger().info(
                    f'TURN_RIGHT | Target: {math.degrees(self.turn_target_yaw):.1f}°, Current: {math.degrees(self.yaw):.1f}°, Diff: {math.degrees(angle_diff):.1f}°',
                    throttle_duration_sec=1.0
                )

        # Publish command
        self.publisher_.publish(cmd)

    def _stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

    def emergency_stop(self):
        """Emergency stop"""
        self.is_active = False
        self._stop_robot()
        self.get_logger().error('EMERGENCY STOP')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutting down controller')
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
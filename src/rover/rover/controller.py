import rclpy
import math
import signal
import sys
import time
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
        self.declare_parameter('max_speed', 0.11)
        self.declare_parameter('max_turn_rate', 0.75)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('is_active', False)
        self.declare_parameter('turn_tolerance', 0.15)  # 8.6°
        self.declare_parameter('min_turn_rate', 0.3)   # Minimum turn speed

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.is_active = self.get_parameter('is_active').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value
        self.min_turn_rate = self.get_parameter('min_turn_rate').value

        # Subscriptions - USE FUSED ODOMETRY (encoder position + IMU yaw)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/fused', self.odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Control timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # State from fused odometry (position from encoder, yaw from IMU)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # ← FROM IMU (via /odometry/fused)
        self.linear_velocity = 0.0

        # IMU direct (for angular velocity)
        self.imu_angular_velocity = 0.0

        # LiDAR state
        self.closest_range_front = float('inf')
        self.closest_range_left = float('inf')
        self.closest_range_right = float('inf')

        # Sensor flags
        self.odom_received = False
        self.imu_received = False
        self.scan_received = False
        self.last_odom_time = None
        self.odom_timeout = 2.0

        # State machine
        self.state = 'FORWARD'
        self.turn_target_yaw = None
        self.turn_start_yaw = None
        self.turn_start_time = None
        self.post_turn_deadline = 0.0  # Cooldown after turn

        self.get_logger().info(
            f'Controller initialized:\n'
            f'  Using /odometry/fused (encoder + IMU)\n'
            f'  Turn tolerance: {math.degrees(self.turn_tolerance):.1f}°\n'
            f'  Min turn rate: {self.min_turn_rate} rad/s'
        )

    def odom_callback(self, msg):
        """Extract fused odometry: encoder position + IMU yaw"""
        if not self.odom_received:
            self.get_logger().info('Fused odometry received!')
            self.odom_received = True

        self.last_odom_time = self.get_clock().now()

        # Position from encoder
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Yaw from IMU (integrated gyro) ← ACCURATE FOR TURNING!
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)

        # Linear velocity from encoder
        self.linear_velocity = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        """Extract angular velocity from raw IMU"""
        if not self.imu_received:
            self.get_logger().info('IMU data received!')
            self.imu_received = True
        
        self.imu_angular_velocity = msg.angular_velocity.z

    def scan_callback(self, msg):
        """Process LiDAR data"""
        if not self.scan_received:
            self.get_logger().info(f'LiDAR scan received! {len(msg.ranges)} points')
            self.scan_received = True

        if len(msg.ranges) == 0:
            return

        # Check cooldown period
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.post_turn_deadline:
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
            if -20 <= angle_deg <= 20:
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

        # Trigger turn if obstacle detected while moving forward
        if self.state == 'FORWARD' and self.closest_range_front < self.obstacle_threshold:
            # Determine turn direction (inverted for rear-facing LiDAR)
            turn_direction = -1 if self.closest_range_left > self.closest_range_right else 1

            self.turn_start_yaw = self.yaw  # ← Current IMU yaw
            self.turn_start_time = self.get_clock().now().nanoseconds / 1e9

            # Calculate target with cardinal snapping
            nominal_target = self.yaw + (math.pi / 2.0 * turn_direction)
            self.turn_target_yaw = self._snap_to_cardinal_yaw(nominal_target)

            self.state = 'TURN'

            direction_str = "RIGHT (-90°)" if turn_direction == 1 else "LEFT (+90°)"
            self.get_logger().info(
                f'Obstacle {self.closest_range_front:.2f}m! L={self.closest_range_left:.2f}m R={self.closest_range_right:.2f}m\n'
                f'Turning {direction_str} from {math.degrees(self.yaw):.1f}° to {math.degrees(self.turn_target_yaw):.1f}°'
            )

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _shortest_angular_dist(self, from_angle, to_angle):
        """Calculate shortest angular distance between two angles"""
        return self._normalize_angle(to_angle - from_angle)

    def _snap_to_cardinal_yaw(self, yaw):
        """Snap yaw to nearest cardinal direction (0°, 90°, 180°, -90°)
        Prevents drift accumulation"""
        normalized_yaw = self._normalize_angle(yaw)
        step = math.pi / 2.0
        N = round(normalized_yaw / step)
        snapped = self._normalize_angle(N * step)
        
        self.get_logger().info(
            f'Cardinal snap: {math.degrees(yaw):.1f}° → {math.degrees(snapped):.1f}°'
        )
        
        return snapped

    def timer_callback(self):
        """Main control loop"""
        # Check if active
        if not self.is_active:
            self.get_logger().warn('Controller INACTIVE', throttle_duration_sec=5.0)
            self._stop_robot()
            return

        # Check sensor data
        if not self.odom_received:
            self.get_logger().warn('Waiting for fused odometry...', throttle_duration_sec=2.0)
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
            self.get_logger().warn(f'Odometry timeout ({time_since_odom:.2f}s)', throttle_duration_sec=2.0)
            self._stop_robot()
            return

        cmd = Twist()

        if self.state == 'FORWARD':
            # Move forward
            cmd.linear.x = -self.max_speed
            cmd.angular.z = 0.0
            
            self.get_logger().info(
                f'FORWARD | Front: {self.closest_range_front:.2f}m | Yaw: {math.degrees(self.yaw):.1f}°',
                throttle_duration_sec=2.0
            )

        elif self.state == 'TURN':
            # Check turn timeout
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self.turn_start_time and (now_sec - self.turn_start_time) > 10.0:
                self.get_logger().warn('Turn timeout (10s)! Forcing FORWARD.')
                self.state = 'FORWARD'
                self.turn_target_yaw = None
                self.turn_start_time = None
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Calculate angle error using IMU yaw
                angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)

                # Check if turn complete (using IMU yaw!)
                if abs(angle_diff) < self.turn_tolerance:
                    angle_turned = abs(self._shortest_angular_dist(self.turn_start_yaw, self.yaw))
                    self.get_logger().info(
                        f'✓ Turn complete! Turned {math.degrees(angle_turned):.1f}°\n'
                        f'  Start yaw: {math.degrees(self.turn_start_yaw):.1f}°\n'
                        f'  Target yaw: {math.degrees(self.turn_target_yaw):.1f}°\n'
                        f'  Final yaw: {math.degrees(self.yaw):.1f}° (IMU-confirmed!)\n'
                        f'  Error: {math.degrees(angle_diff):.2f}°'
                    )
                    
                    # Switch to FORWARD with cooldown
                    self.state = 'FORWARD'
                    self.turn_target_yaw = None
                    self.turn_start_time = None
                    
                    # Set cooldown (750ms to let robot stabilize)
                    self.post_turn_deadline = now_sec + 0.75
                    
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    # Continue turning - proportional control
                    kp = 2.5  # Proportional gain
                    angular_z = kp * angle_diff
                    
                    # Clamp to max turn rate
                    angular_z = max(-self.max_turn_rate, min(self.max_turn_rate, angular_z))
                    
                    # Enforce minimum turn rate (prevents stalling from wheel slip)
                    if abs(angular_z) < self.min_turn_rate:
                        angular_z = self.min_turn_rate * (1 if angular_z > 0 else -1)
                    
                    cmd.linear.x = 0.0
                    cmd.angular.z = angular_z
                    
                    self.get_logger().info(
                        f'TURN | IMU Yaw: {math.degrees(self.yaw):.1f}° → Target: {math.degrees(self.turn_target_yaw):.1f}° '
                        f'| Error: {math.degrees(angle_diff):.1f}° | ω: {angular_z:.2f} rad/s',
                        throttle_duration_sec=0.3
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
        print('⚠️  CONTROLLER: Emergency stop!')
        cmd = Twist()
        for i in range(10):
            self.publisher_.publish(cmd)
            time.sleep(0.02)
        print('✓ Controller stopped')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    def signal_handler(sig, frame):
        print('\n⚠️  CTRL+C detected! Stopping robot...')
        node.emergency_stop()
        print('✓ Safe to exit')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
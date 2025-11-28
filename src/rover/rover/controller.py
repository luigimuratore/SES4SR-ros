import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import math
import tf_transformations
import signal
import sys
import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('max_turn_rate', 0.8)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('is_active', True)
        self.declare_parameter('turn_tolerance', 0.15)  # 11.5° tolerance

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.is_active = self.get_parameter('is_active').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value
        
        self.get_logger().info(f'Parameters loaded: max_speed={self.max_speed}, '
                              f'max_turn_rate={self.max_turn_rate}, '
                              f'turn_tolerance={math.degrees(self.turn_tolerance):.1f}°')

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Obstacle detection
        self.closest_range_front = float('inf')
        self.closest_range_left = float('inf')
        self.closest_range_right = float('inf')
        
        # Odometry tracking (IMU-based yaw from EKF)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Sensor data timestamps
        self.last_odom_time = None
        self.last_imu_time = None
        self.odom_received = False
        self.imu_received = False
        self.scan_received = False
        self.odom_timeout = 2.0
        
        # State machine
        self.state = 'FORWARD'
        self.turn_target_yaw = None
        self.turn_start_yaw = None
        
        # Post-turn cooldown (from lab03) - prevents rapid re-triggering
        self.post_turn_deadline = 0.0
        
        self.get_logger().info('Controller initialized and ready')

    def odom_callback(self, msg):
        """Extract position and yaw from fused odometry (encoder + IMU)"""
        if not self.odom_received:
            self.get_logger().info('First filtered odometry received!')
            self.odom_received = True
            
        self.last_odom_time = self.get_clock().now()
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Get yaw from fused odometry (IMU-based) - corrects for wheel slip!
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)
        
        self.linear_velocity = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        """Extract angular velocity from IMU"""
        if not self.imu_received:
            self.get_logger().info('First IMU message received!')
            self.imu_received = True
            
        self.last_imu_time = self.get_clock().now()
        self.angular_velocity = msg.angular_velocity.z

    def scan_callback(self, msg):
        """Process LiDAR data"""
        if not self.scan_received:
            self.get_logger().info(f'First scan received! {len(msg.ranges)} points')
            self.scan_received = True
            
        if len(msg.ranges) == 0:
            return
        
        # Check cooldown period (from lab03)
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
            
            if -30 <= angle_deg <= 30:
                front_ranges.append(r)
            elif 30 < angle_deg <= 90:
                left_ranges.append(r)
            elif -90 <= angle_deg < -30:
                right_ranges.append(r)
        
        self.closest_range_front = min(front_ranges) if front_ranges else float('inf')
        self.closest_range_left = min(left_ranges) if left_ranges else float('inf')
        self.closest_range_right = min(right_ranges) if right_ranges else float('inf')
        
        # Trigger turn if obstacle detected while FORWARD
        if self.state == 'FORWARD' and self.closest_range_front < self.obstacle_threshold:
            if self.yaw is None:
                self.get_logger().warn('Obstacle detected but no odometry yet')
                return
            
            # Determine turn direction
            turn_direction = -1 if self.closest_range_left > self.closest_range_right else 1
            
            # Record starting yaw
            self.turn_start_yaw = self.yaw
            
            # Calculate target with cardinal snapping (key feature from lab03!)
            nominal_target = self.yaw + (math.pi / 2.0 * turn_direction)
            self.turn_target_yaw = self._snap_to_cardinal_yaw(nominal_target)
            
            self.state = 'TURN'
            
            direction_str = "LEFT (+90°)" if turn_direction == 1 else "RIGHT (-90°)"
            self.get_logger().info(
                f'Obstacle at {self.closest_range_front:.2f}m! '
                f'Turning {direction_str} from {math.degrees(self.yaw):.1f}° '
                f'to {math.degrees(self.turn_target_yaw):.1f}° (snapped)'
            )

    def _snap_to_cardinal_yaw(self, yaw):
        """Snap to nearest cardinal direction (0°, 90°, 180°, -90°)
        This prevents drift accumulation from lab03"""
        normalized_yaw = self._normalize_angle(yaw)
        step = math.pi / 2.0  # 90 degrees
        
        # Round to nearest multiple of 90°
        N = round(normalized_yaw / step)
        snapped_yaw = N * step
        
        return self._normalize_angle(snapped_yaw)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _shortest_angular_dist(self, from_angle, to_angle):
        """Calculate shortest angular distance"""
        diff = to_angle - from_angle
        return self._normalize_angle(diff)

    def timer_callback(self):
        """Main control loop"""
        if not self.is_active:
            self.get_logger().warn('Controller is INACTIVE', throttle_duration_sec=5.0)
            self._stop_robot()
            return
        
        if not self.odom_received or not self.imu_received or not self.scan_received:
            self.get_logger().warn('Waiting for sensors...', throttle_duration_sec=2.0)
            self._stop_robot()
            return
        
        # Check odometry freshness
        time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if time_since_odom > self.odom_timeout:
            self.get_logger().warn(f'Odometry timeout ({time_since_odom:.2f}s)', throttle_duration_sec=2.0)
            self._stop_robot()
            return
        
        cmd = Twist()
        
        if self.state == 'FORWARD':
            # Drive straight
            cmd.linear.x = -self.max_speed
            cmd.angular.z = 0.0
        
        elif self.state == 'TURN':
            if self.turn_target_yaw is None or self.yaw is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Calculate error
                angle_diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
                
                # Check if turn complete
                if abs(angle_diff) < self.turn_tolerance:
                    # Turn complete!
                    angle_turned = abs(self._shortest_angular_dist(self.turn_start_yaw, self.yaw))
                    self.get_logger().info(
                        f'Turn complete! Turned {math.degrees(angle_turned):.1f}°, '
                        f'Final yaw: {math.degrees(self.yaw):.1f}°'
                    )
                    
                    # Switch to FORWARD with cooldown period
                    self.state = 'FORWARD'
                    self.turn_target_yaw = None
                    self.turn_start_yaw = None
                    
                    # Set cooldown (from lab03) - prevents immediate re-turn
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                    self.post_turn_deadline = now_sec + 0.5  # 500ms cooldown
                    
                    cmd.linear.x = self.max_speed
                    cmd.angular.z = 0.0
                else:
                    # Continue turning with proportional control
                    # Clamp to max turn rate
                    angular_z = max(-self.max_turn_rate, 
                                  min(self.max_turn_rate, 2.0 * angle_diff))
                    
                    cmd.linear.x = 0.0
                    cmd.angular.z = angular_z
                    
                    self.get_logger().info(
                        f'Turning: current={math.degrees(self.yaw):.1f}°, '
                        f'target={math.degrees(self.turn_target_yaw):.1f}°, '
                        f'error={math.degrees(angle_diff):.1f}°',
                        throttle_duration_sec=0.5
                    )
        
        self.publisher_.publish(cmd)
        
        # Debug info when not turning
        if self.state == 'FORWARD':
            self.get_logger().info(
                f'State: FORWARD, Front: {self.closest_range_front:.2f}m, '
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

    def emergency_stop(self):
        """Emergency stop"""
        print('EMERGENCY STOP - Stopping motors!')
        cmd = Twist()
        for i in range(10):
            self.publisher_.publish(cmd)
            time.sleep(0.02)
        print('Motors stopped!')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    def signal_handler(sig, frame):
        print('\n⚠️  CTRL+C detected! Stopping robot...')
        node.emergency_stop()
        print('✓ Safe to exit now')
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
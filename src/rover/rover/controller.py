import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math 

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.20)
        self.declare_parameter('max_turn_rate', 1.5)
        self.declare_parameter('is_active', False)  # Start inactive for manual testing
        self.declare_parameter('use_odometry', False)  # NEW: disable odometry requirement

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.is_active = self.get_parameter('is_active').value
        self.use_odometry = self.get_parameter('use_odometry').value
        
        self.get_logger().info(f'Parameters: max_speed={self.max_speed}, max_turn_rate={self.max_turn_rate}, is_active={self.is_active}, use_odometry={self.use_odometry}')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # store latest closest obstacle info
        self.closest_range_front = float('inf')
        
        # Odometry-based state (only used if use_odometry=True)
        self.last_odom_time = None
        self.odom_timeout = 0.5
        self.yaw = 0.0  # Default yaw if no odometry
        self.turn_target_yaw = None
        self.state = 'FORWARD'
        self.obstacle_threshold = 0.5
        self.turn_tolerance = 0.05
        self.post_turn_deadline = 0.0
        
        # Simple obstacle avoidance without odometry
        self.turn_duration = 2.0  # seconds to turn when obstacle detected
        self.turn_start_time = None

    def scan_callback(self, msg):
        # Check front sector for obstacles
        front_sector = msg.ranges[165:195]  # ~30 degree cone in front
        
        valid_ranges = [r for r in front_sector if math.isfinite(r) and r > msg.range_min]
        self.closest_range_front = min(valid_ranges) if valid_ranges else float('inf')
        
        # Log obstacle distance
        if self.closest_range_front < self.obstacle_threshold:
            self.get_logger().info(f'Obstacle detected at {self.closest_range_front:.2f}m')

    def timer_callback(self):
        msg = Twist()
        
        if not self.is_active:
            # Controller is inactive - just pass through or stop
            self.publisher_.publish(msg)
            return
        
        # Simple obstacle avoidance (no odometry needed)
        if self.turn_start_time is not None:
            # Currently turning
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.turn_start_time
            if elapsed < self.turn_duration:
                # Keep turning
                msg.linear.x = 0.0
                msg.angular.z = float(self.max_turn_rate)
                self.get_logger().info(f'Turning... {elapsed:.1f}s / {self.turn_duration}s')
            else:
                # Turn complete
                self.turn_start_time = None
                msg.linear.x = float(self.max_speed)
                msg.angular.z = 0.0
                self.get_logger().info('Turn complete, moving forward')
        elif self.closest_range_front < self.obstacle_threshold:
            # Obstacle detected - start turning
            self.turn_start_time = self.get_clock().now().nanoseconds / 1e9
            msg.linear.x = 0.0
            msg.angular.z = float(self.max_turn_rate)
            self.get_logger().warn(f'Obstacle at {self.closest_range_front:.2f}m - starting turn')
        else:
            # No obstacle - move forward
            msg.linear.x = float(self.max_speed)
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
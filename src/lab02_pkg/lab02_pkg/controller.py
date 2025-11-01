import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations 
import math 

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # declare parameters
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('max_turn_rate', 1.5)
        self.declare_parameter('is_active', True)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.is_active = self.get_parameter('is_active').value
        self.get_logger().info(f'Parameters loaded: max_speed={self.max_speed}, max_turn_rate={self.max_turn_rate}, is_active={self.is_active}')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.groundtruth_sub = self.create_subscription(Odometry, '/ground_truth', self.groundtruth_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # store latest closest obstacle info
        self.closest_range_front = float('inf')
        
        # Add timestamp tracking for odometry
        self.last_odom_time = None
        self.odom_timeout = 0.5  # seconds
        
        # avoid wall: FORWARD or TURN
        self.state = 'FORWARD'
        self.yaw = None
        self.turn_target_yaw = None
        self.obstacle_threshold = 0.5
        self.turn_tolerance = 0.1  # Increase tolerance for real robot
        self.post_turn_deadline = 0.0



    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat_list)
        
        # Track when we last received odometry
        self.last_odom_time = self.get_clock().now()
        
        # Log yaw during turns for debugging
        if self.state == 'TURN' and self.turn_target_yaw is not None:
            diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
            self.get_logger().debug(f'TURN: current_yaw={self.yaw:.2f}, target={self.turn_target_yaw:.2f}, diff={diff:.2f}')

    def scan_callback(self, msg):
        
        # 1 Check only the front sector for obstacles -> LiDar -> assuming 360 points, index 0 is forward.
        front_sector = msg.ranges[:16] + msg.ranges[345:] # Indices 0-15 and 345-359
        
        valid_ranges = [r for r in front_sector if math.isfinite(r) and r > msg.range_min]  # exclude invalid values (inf, nan, and below min range)
        
        self.closest_range_front = min(valid_ranges) if valid_ranges else float('inf')
        
        # enforce a waiting period to drive forward before allowing a new turn
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.post_turn_deadline:
            return
            

        # 2 TURN if an obstacle is detected while FORWARD
        if self.state == 'FORWARD' and self.closest_range_front < self.obstacle_threshold:
            if self.yaw is None:
                self.get_logger().info('Obstacle detected but no odom yet — stopping')
                self.publisher_.publish(Twist())
                return
            
            turn_direction = self._determine_clearest_side(msg.ranges)    # Determine the clearest side for a 90-degree turn
            
            nominal_target = self.yaw + (math.pi / 2.0 * turn_direction)     # Calculate the nominal 90-degree turn
            self.turn_target_yaw = self._snap_to_cardinal_yaw(nominal_target) # avoid drift by snapping to cardinal directions

            self.state = 'TURN'
            
            direction_str = "LEFT (+90 deg)" if turn_direction == 1 else "RIGHT (-90 deg)"
            self.get_logger().warn(
                f'Triggering TURN: Front closest={self.closest_range_front:.2f}m. Turning {direction_str} to target_yaw={self.turn_target_yaw:.2f} rad'
            )
            


    def _determine_clearest_side(self, ranges):
        left_ranges = ranges[45:136]         # 90-degree sector Left (approx 45 to 135 degrees)
        right_ranges = ranges[225:316]         # 90-degree sector Right (approx 225 to 315 degrees)
        
        MAX_RANGE = 3.5         # Get the max range value (3.5 m)

        def get_valid_ranges(r_list):
            return [MAX_RANGE if math.isinf(r) else r for r in r_list if math.isfinite(r) or math.isinf(r)]   # MAX_RANGE if inf 
             
        def get_avg(r_list):
            valid_list = get_valid_ranges(r_list)
            
            if not valid_list:
                return 0.0
                
            return sum(valid_list) / len(r_list)             # Calculate average based on the length of the original sector (91 elements)

        left_avg = get_avg(left_ranges)
        right_avg = get_avg(right_ranges)
        
        return 1 if left_avg >= right_avg else -1  # 1 for left, -1 for right
    


    def _snap_to_cardinal_yaw(self, yaw):
        
        normalized_yaw = self._normalize_angle(yaw)         # Normalize to [-pi, pi] first
        step = math.pi / 2.0         # Cardinal directions are multiples of pi/2 (~1.5708 rad)
        
        N = round(normalized_yaw / step)         # Calculate steps (N) of pi/2 away we are from 0
        snapped_yaw = N * step         # Snap the yaw to the nearest multiple of step

        return self._normalize_angle(snapped_yaw)



    def timer_callback(self):
        msg = Twist()
        if not self.is_active:
            self.publisher_.publish(msg)
            return
        
        # Check if odometry is fresh
        now = self.get_clock().now()
        if self.last_odom_time is None:
            self.get_logger().warn('No odometry received yet, stopping')
            self.publisher_.publish(msg)
            return
            
        odom_age = (now - self.last_odom_time).nanoseconds / 1e9
        if odom_age > self.odom_timeout:
            self.get_logger().warn(f'Odometry stale ({odom_age:.2f}s), stopping')
            self.publisher_.publish(msg)
            return

        if self.state == 'FORWARD':             # Drive straight until obstacle is detected
            msg.linear.x = float(self.max_speed) 
            msg.angular.z = 0.0

        elif self.state == 'TURN':
            if self.turn_target_yaw is None or self.yaw is None:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                diff = self._shortest_angular_dist(self.yaw, self.turn_target_yaw)
                ang = max(-float(self.max_turn_rate), min(float(self.max_turn_rate), 2.0 * diff))
                
                if abs(diff) < self.turn_tolerance:
                    self.state = 'FORWARD'
                    self.turn_target_yaw = None
                    
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                    self.post_turn_deadline = now_sec + 0.5
                    
                    msg.linear.x = float(self.max_speed)
                    msg.angular.z = 0.0
                    self.get_logger().info(f'Turn complete at yaw={self.yaw:.2f}, starting FORWARD')
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = ang
                    self.get_logger().info(f'Turning: diff={diff:.2f} rad, angular.z={ang:.2f}')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Controller state={self.state}, cmd linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')


    
    def groundtruth_callback(self, msg):
        pass    



    def _normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))



    def _shortest_angular_dist(self, from_angle, to_angle):
        diff = self._normalize_angle(to_angle - from_angle)
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

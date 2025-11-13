import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from std_msgs.msg import String

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber to encoder data from arduino_bridge
        self.encoder_sub = self.create_subscription(
            String,
            '/encoder_data',
            self.encoder_callback,
            10
        )
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.065)  # meters
        self.declare_parameter('wheel_base', 0.30)     # meters between left/right wheels
        self.declare_parameter('ticks_per_rev', 1600)  # encoder ticks per wheel revolution
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_encoder1 = 0
        self.last_encoder2 = 0
        self.last_encoder4 = 0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info(f'Odometry publisher initialized with wheel_radius={self.wheel_radius}, wheel_base={self.wheel_base}')
    
    def encoder_callback(self, msg):
        try:
            # Parse encoder data: "E,encoder1,encoder2,encoder4,timestamp"
            parts = msg.data.split(',')
            if parts[0] != 'E' or len(parts) != 5:
                return
            
            encoder1 = int(parts[1])
            encoder2 = int(parts[2])
            encoder4 = int(parts[3])
            
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt < 0.001:  # Avoid division by zero
                return
            
            # Calculate wheel displacements using 3 encoders
            # Left side: only encoder1 (encoder3 is broken)
            delta_left = (encoder1 - self.last_encoder1) * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            
            # Right side: average of encoder2 and encoder4
            delta_right_2 = (encoder2 - self.last_encoder2) * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            delta_right_4 = (encoder4 - self.last_encoder4) * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            delta_right = (delta_right_2 + delta_right_4) / 2.0
            
            # Calculate robot motion
            delta_distance = (delta_left + delta_right) / 2.0
            delta_theta = (delta_right - delta_left) / self.wheel_base
            
            # Update pose
            self.theta += delta_theta
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            
            # Calculate velocities
            v = delta_distance / dt
            omega = delta_theta / dt
            
            # Publish odometry
            self.publish_odometry(current_time, v, omega)
            
            # Update last values
            self.last_encoder1 = encoder1
            self.last_encoder2 = encoder2
            self.last_encoder4 = encoder4
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error processing encoder data: {e}')
    
    def publish_odometry(self, current_time, v, omega):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
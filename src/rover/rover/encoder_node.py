import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import math

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.065)  # meters
        self.declare_parameter('wheel_base', 0.30)     # meters between left/right wheels
        self.declare_parameter('ticks_per_rev', 1600)  # encoder ticks per wheel revolution
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Connected to Arduino on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_encoder1 = 0
        self.last_encoder2 = 0
        self.last_time = self.get_clock().now()
        self.first_reading = True
        
        # Timer to read from Arduino
        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz
        
        self.get_logger().info(f'Encoder node initialized: wheel_radius={self.wheel_radius}m, wheel_base={self.wheel_base}m')
    
    def cmd_vel_callback(self, msg):
        # Send command to Arduino: "linear,angular"
        command = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        try:
            self.serial.write(command.encode())
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')
    
    def read_serial(self):
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # Check if it's encoder data: "E,encoder1,encoder2"
                if line.startswith('E,'):
                    self.process_encoder_data(line)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')
    
    def process_encoder_data(self, data):
        try:
            # Parse encoder data: "E,encoder1,encoder2"
            parts = data.split(',')
            if len(parts) != 3:
                return
            
            encoder1 = int(parts[1])  # Left wheel
            encoder2 = int(parts[2])  # Right wheel
            
            # Skip first reading (no previous data for delta)
            if self.first_reading:
                self.last_encoder1 = encoder1
                self.last_encoder2 = encoder2
                self.last_time = self.get_clock().now()
                self.first_reading = False
                return
            
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt < 0.001:  # Avoid division by zero
                return
            
            # Calculate wheel displacements
            delta_encoder1 = encoder1 - self.last_encoder1
            delta_encoder2 = encoder2 - self.last_encoder2
            
            # Convert encoder ticks to linear distance
            delta_left = delta_encoder1 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            delta_right = delta_encoder2 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
            
            # Calculate robot motion (differential drive kinematics)
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
        
        # Covariance (adjust based on your robot's characteristics)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.05  # yaw
        
        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # vyaw
        
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
    
    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
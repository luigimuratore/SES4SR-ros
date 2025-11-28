import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import signal
import sys

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('ticks_per_rev', 1600)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        # Serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder1 = 0
        self.last_encoder2 = 0
        self.last_time = self.get_clock().now()
        self.first_reading = True
        
        # IMU state
        self.imu_yaw = 0.0
        self.last_imu_time = None
        self.imu_initialized = False
        
        # Timer
        self.timer = self.create_timer(0.01, self.read_serial)
        
        self.get_logger().info('Odom node initialized')
    
    def cmd_vel_callback(self, msg):
        command = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        try:
            self.serial.write(command.encode())
            self.serial.flush()  # Force write immediately
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def emergency_stop(self):
        """Send stop command directly to Arduino"""
        print('ODOM NODE: Emergency stop - sending to Arduino!')
        stop_command = "0.000,0.000\n"
        
        try:
            # Send stop command 20 times rapidly
            for i in range(20):
                self.serial.write(stop_command.encode())
                self.serial.flush()
                time.sleep(0.01)
            
            print('ODOM NODE: Stop commands sent to Arduino!')
        except Exception as e:
            print(f'ODOM NODE: Error sending stop: {e}')
    
    def read_serial(self):
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                if line.startswith('D,'):
                    self.process_combined_data(line)
                    
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
    
    def process_combined_data(self, data):
        try:
            parts = data.split(',')
            if len(parts) != 9:
                return
            
            encoder1 = int(parts[1])
            encoder2 = int(parts[2])
            
            ax_mg = float(parts[3])
            ay_mg = float(parts[4])
            az_mg = float(parts[5])
            gx_dps = float(parts[6])
            gy_dps = float(parts[7])
            gz_dps = float(parts[8])
            
            current_time = self.get_clock().now()
            
            # Process IMU FIRST
            self.process_imu(ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps, current_time)
            
            # Then process odometry
            self.process_odometry(encoder1, encoder2, current_time)
            
        except Exception as e:
            self.get_logger().error(f'Data processing error: {e}')
    
    def process_imu(self, ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps, current_time):
        # Convert units
        ax = (ax_mg / 1000.0) * 9.80665
        ay = (ay_mg / 1000.0) * 9.80665
        az = (az_mg / 1000.0) * 9.80665
        
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)
        
        # Initialize on first reading
        if not self.imu_initialized:
            self.last_imu_time = current_time
            self.imu_yaw = 0.0
            self.imu_initialized = True
            self.get_logger().info('IMU initialized - starting yaw integration')
            return
        
        # Calculate dt
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        
        # Sanity check on dt
        if dt <= 0 or dt > 1.0:
            self.last_imu_time = current_time
            return
        
        # Integrate gyro for yaw
        self.imu_yaw += gz * dt
        
        # Normalize to [-pi, pi]
        while self.imu_yaw > math.pi:
            self.imu_yaw -= 2 * math.pi
        while self.imu_yaw < -math.pi:
            self.imu_yaw += 2 * math.pi
        
        self.last_imu_time = current_time
        
        # Create IMU message
        msg = Imu()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'
        
        # Orientation from integrated gyro
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.imu_yaw / 2.0)
        msg.orientation.w = math.cos(self.imu_yaw / 2.0)
        
        # VERY LOW covariance = trust IMU strongly
        msg.orientation_covariance[0] = 0.1
        msg.orientation_covariance[4] = 0.1
        msg.orientation_covariance[8] = 0.0001
        
        # Angular velocity
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.0001
        
        # Linear acceleration
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        
        self.imu_pub.publish(msg)
    
    def process_odometry(self, encoder1, encoder2, current_time):
        if self.first_reading:
            self.last_encoder1 = encoder1
            self.last_encoder2 = encoder2
            self.last_time = current_time
            self.first_reading = False
            return
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001:
            return
        
        delta_encoder1 = encoder1 - self.last_encoder1
        delta_encoder2 = encoder2 - self.last_encoder2
        
        delta_left = delta_encoder1 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        delta_right = delta_encoder2 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base
        
        self.theta += delta_theta
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        
        v = delta_distance / dt
        
        self.publish_odometry(current_time, v)
        
        self.last_encoder1 = encoder1
        self.last_encoder2 = encoder2
        self.last_time = current_time
    
    def publish_odometry(self, current_time, v):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Identity quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Covariance
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 1000000
        
        # ONLY linear velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 1000000
        
        self.odom_pub.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        self.get_logger().info('Closing serial connection...')
        self.emergency_stop()
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    
    def signal_handler(sig, frame):
        """Handle CTRL+C"""
        print('\n⚠️  ODOM NODE: CTRL+C detected! Stopping motors via serial...')
        node.emergency_stop()
        print('✓ ODOM NODE: Stop commands sent')
        
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handler
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
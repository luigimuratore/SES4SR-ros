import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        
        # Fusion thresholds
        self.declare_parameter('min_velocity_threshold', 0.005)  # Lowered to 0.005 m/s
        self.declare_parameter('min_encoder_ticks', 2)          # Minimum ticks to consider moving
        self.declare_parameter('min_gyro_threshold', 0.05)      # Increased to 0.05 rad/s (~2.9°/s)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        self.min_velocity_threshold = self.get_parameter('min_velocity_threshold').value
        self.min_encoder_ticks = self.get_parameter('min_encoder_ticks').value
        self.min_gyro_threshold = self.get_parameter('min_gyro_threshold').value
        
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
        self.odom_encoder_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.odom_fused_pub = self.create_publisher(Odometry, '/odometry/fused', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Encoder state
        self.x = 0.0
        self.y = 0.0
        self.encoder_theta = 0.0
        self.last_encoder1 = 0
        self.last_encoder2 = 0
        self.last_odom_time = self.get_clock().now()
        self.first_encoder_reading = True
        
        # IMU state
        self.imu_yaw = 0.0
        self.last_imu_time = None
        self.imu_initialized = False
        self.imu_raw_gx = 0.0
        self.imu_raw_gy = 0.0
        self.imu_raw_gz = 0.0
        
        # Current velocities (for sensor agreement check)
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Error tracking
        self.error_count = 0
        self.last_error_log = 0.0
        
        # Timer
        self.timer = self.create_timer(0.01, self.read_serial)
        
        # Add stationary tracking
        self.is_robot_moving = False
        
        self.get_logger().info(
            f'Odom node initialized with fusion validation:\n'
            f'  min_velocity_threshold: {self.min_velocity_threshold} m/s\n'
            f'  min_encoder_ticks: {self.min_encoder_ticks} ticks\n'
            f'  min_gyro_threshold: {self.min_gyro_threshold} rad/s ({math.degrees(self.min_gyro_threshold):.1f}°/s)'
        )
    
    def cmd_vel_callback(self, msg):
        """Send velocity commands to Arduino"""
        command = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        try:
            self.serial.write(command.encode())
            self.serial.flush()
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def emergency_stop(self):
        """Send stop command directly to Arduino"""
        print('ODOM NODE: Emergency stop - sending to Arduino!')
        stop_command = "0.000,0.000\n"
        
        try:
            for i in range(20):
                self.serial.write(stop_command.encode())
                self.serial.flush()
                time.sleep(0.01)
            print('ODOM NODE: Stop commands sent to Arduino!')
        except Exception as e:
            print(f'ODOM NODE: Error sending stop: {e}')
    
    def read_serial(self):
        """Read and parse data from Arduino"""
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    return
                
                if line.startswith('D,'):
                    self.process_combined_data(line)
                elif 'READY' in line or 'FAIL' in line:
                    self.get_logger().info(f'Arduino: {line}')
                    
        except Exception as e:
            self.error_count += 1
            now = time.time()
            if now - self.last_error_log > 5.0:
                self.get_logger().error(f'Serial error (count: {self.error_count}): {e}')
                self.last_error_log = now
    
    def process_combined_data(self, data):
        """Parse 'D,encoder1,encoder2,ax,ay,az,gx,gy,gz' format"""
        try:
            parts = data.split(',')
            
            if len(parts) != 9 or parts[0] != 'D':
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
            
            # Process IMU first
            self.process_imu(ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps, current_time)
            
            # Then process encoder
            self.process_odometry(encoder1, encoder2, current_time)
            
        except ValueError:
            pass
        except Exception as e:
            self.get_logger().error(f'Data processing error: {e}')
    
    def process_imu(self, ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps, current_time):
        """Process IMU data - ONLY integrate gyro when robot is actually moving"""
        # Convert units
        ax = (ax_mg / 1000.0) * 9.80665
        ay = (ay_mg / 1000.0) * 9.80665
        az = (az_mg / 1000.0) * 9.80665
        
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)
        
        # Store raw gyro
        self.imu_raw_gx = gx
        self.imu_raw_gy = gy
        self.imu_raw_gz = gz
        
        # Initialize on first reading
        if not self.imu_initialized:
            self.last_imu_time = current_time
            self.imu_yaw = 0.0
            self.imu_initialized = True
            self.get_logger().info('IMU initialized - starting yaw integration')
            return
        
        # Calculate dt
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        
        if 0.001 < dt < 1.0:
            # ⚠️ CRITICAL: Only integrate gyro if robot is actually moving!
            # This prevents drift accumulation when stationary
            if self.is_robot_moving and abs(gz) > self.min_gyro_threshold:
                self.imu_yaw += gz * dt
                self.imu_yaw = math.atan2(math.sin(self.imu_yaw), math.cos(self.imu_yaw))
            # else: Robot is stationary - DO NOT update yaw (prevents drift!)
        
        self.last_imu_time = current_time
        
        # Publish raw IMU message (always publish raw data)
        msg = Imu()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'
        
        # Orientation from integrated gyro (frozen when stationary)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.imu_yaw / 2.0)
        msg.orientation.w = math.cos(self.imu_yaw / 2.0)
        
        msg.orientation_covariance[0] = 0.1
        msg.orientation_covariance[4] = 0.1
        msg.orientation_covariance[8] = 0.0001
        
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.0001
        
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        
        self.imu_pub.publish(msg)
    
    def process_odometry(self, encoder1, encoder2, current_time):
        """Process encoder with STRICT stationary detection"""
        if self.first_encoder_reading:
            self.last_encoder1 = encoder1
            self.last_encoder2 = encoder2
            self.last_odom_time = current_time
            self.first_encoder_reading = False
            self.get_logger().info(f'First encoder reading: E1={encoder1}, E2={encoder2}')
            return
        
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        if dt < 0.001:
            return
        
        # Calculate encoder deltas
        delta_encoder1 = encoder1 - self.last_encoder1
        delta_encoder2 = encoder2 - self.last_encoder2
        
        # ⚠️ CRITICAL: Check if BOTH encoders show movement
        # If neither encoder moved, robot is STATIONARY
        if abs(delta_encoder1) < self.min_encoder_ticks and abs(delta_encoder2) < self.min_encoder_ticks:
            self.is_robot_moving = False
            
            # Robot is stationary - publish zero velocity
            self.current_linear_velocity = 0.0
            self.current_angular_velocity = 0.0
            
            self.publish_odom_encoder(current_time, 0.0)
            self.publish_odom_fused(current_time, 0.0)
            
            # Update time but not position
            self.last_odom_time = current_time
            
            return  # Don't update position or yaw!
        
        # Robot IS moving - proceed with normal odometry
        self.is_robot_moving = True
        
        # Convert to distance (meters)
        delta_left = delta_encoder1 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        delta_right = delta_encoder2 * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # Calculate movement
        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta_encoder = (delta_right - delta_left) / self.wheel_base
        
        # Calculate velocities
        linear_velocity = delta_distance / dt
        angular_velocity_encoder = delta_theta_encoder / dt
        
        # Store current velocities
        self.current_linear_velocity = linear_velocity
        self.current_angular_velocity = self.imu_raw_gz
        
        # Update position (robot is confirmed moving)
        self.encoder_theta += delta_theta_encoder
        self.x += delta_distance * math.cos(self.encoder_theta)
        self.y += delta_distance * math.sin(self.encoder_theta)
        
        # Publish both odometry messages
        self.publish_odom_encoder(current_time, linear_velocity)
        self.publish_odom_fused(current_time, linear_velocity)
        
        # Store for next iteration
        self.last_encoder1 = encoder1
        self.last_encoder2 = encoder2
        self.last_odom_time = current_time
    
    def publish_odom_encoder(self, current_time, v):
        """Publish encoder-only odometry"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 1000000
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 1000000
        
        self.odom_encoder_pub.publish(odom)
    
    def publish_odom_fused(self, current_time, v):
        """Publish FUSED odometry: encoder position + IMU yaw (validated)"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position from encoder (only updated when moving)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Yaw from IMU (only updated when turning)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.imu_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.imu_yaw / 2.0)
        
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        
        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.imu_raw_gz if abs(self.imu_raw_gz) > self.min_gyro_threshold else 0.0
        
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.001
        
        self.odom_fused_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.imu_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.imu_yaw / 2.0)
        
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
        print('\n⚠️  ODOM NODE: Emergency stop...')
        node.emergency_stop()
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
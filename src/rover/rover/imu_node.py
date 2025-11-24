#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import serial
import math
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

class IMUArduinoNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise
        
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz
        
        # Publish static transform
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        
        self.get_logger().info("Arduino IMU node started")
    
    def publish_static_transform(self):
        """Publish static transform from base_link to imu_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        # Position offset (adjust to your IMU mounting position)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above base
        
        # No rotation offset
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def update(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Skip status messages
                if line.startswith('CALIBRATING') or line.startswith('IMU_'):
                    self.get_logger().info(line)
                    return
                
                # Parse "roll pitch yaw" format
                parts = line.split()
                if len(parts) != 3:
                    return
                
                roll_deg = float(parts[0])
                pitch_deg = float(parts[1])
                yaw_deg = float(parts[2])
                
                # Convert degrees to radians
                roll = math.radians(roll_deg)
                pitch = math.radians(pitch_deg)
                yaw = math.radians(yaw_deg)
                
                # Convert Euler angles to quaternion
                quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                
                # Create and publish IMU message
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                
                msg.orientation.x = quaternion[0]
                msg.orientation.y = quaternion[1]
                msg.orientation.z = quaternion[2]
                msg.orientation.w = quaternion[3]
                
                # Orientation covariance (tune these values)
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.05  # yaw less certain
                
                # Set linear acceleration to zero (not available from Arduino)
                msg.linear_acceleration.x = 0.0
                msg.linear_acceleration.y = 0.0
                msg.linear_acceleration.z = 0.0
                msg.linear_acceleration_covariance[0] = -1.0  # Unknown
                
                # Set angular velocity to zero (not available from Arduino)
                msg.angular_velocity.x = 0.0
                msg.angular_velocity.y = 0.0
                msg.angular_velocity.z = 0.0
                msg.angular_velocity_covariance[0] = -1.0  # Unknown
                
                self.pub.publish(msg)
                
                # Log occasionally
                if self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                    self.get_logger().info(
                        f"Roll: {roll_deg:6.1f}° Pitch: {pitch_deg:6.1f}° Yaw: {yaw_deg:6.1f}°",
                        throttle_duration_sec=0.5
                    )
                    
        except ValueError as e:
            self.get_logger().warn(f"Parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")
    
    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = IMUArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
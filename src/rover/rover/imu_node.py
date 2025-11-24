#!/usr/bin/env python3
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUNode(Node):
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

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

        self.get_logger().info("Arduino IMU node started")

    def update(self):
        try:
            if self.ser.in_waiting <= 0:
                return

            line = self.ser.readline().decode('utf-8').strip()

            # Skip debug lines like "IMU_READY"
            if not line or not (line[0].isdigit() or line[0] == '-' ):
                return

            parts = line.split()
            if len(parts) != 6:
                return

            ax_mg = float(parts[0])
            ay_mg = float(parts[1])
            az_mg = float(parts[2])
            gx_dps = float(parts[3])
            gy_dps = float(parts[4])
            gz_dps = float(parts[5])

            # mg -> m/s^2
            ax = (ax_mg / 1000.0) * 9.80665
            ay = (ay_mg / 1000.0) * 9.80665
            az = (az_mg / 1000.0) * 9.80665

            # dps -> rad/s
            gx = math.radians(gx_dps)
            gy = math.radians(gy_dps)
            gz = math.radians(gz_dps)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # orientation unknown (EKF can estimate it)
            msg.orientation_covariance[0] = -1.0

            # angular velocity
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance[0] = 0.02 ** 2
            msg.angular_velocity_covariance[4] = 0.02 ** 2
            msg.angular_velocity_covariance[8] = 0.02 ** 2

            # linear acceleration
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance[0] = 0.2 ** 2
            msg.linear_acceleration_covariance[4] = 0.2 ** 2
            msg.linear_acceleration_covariance[8] = 0.2 ** 2

            self.pub.publish(msg)

        except ValueError as e:
            self.get_logger().warn(f"Parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

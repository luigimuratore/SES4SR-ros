import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
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
        
        # Publisher for encoder data
        self.encoder_pub = self.create_publisher(String, '/encoder_data', 10)
        
        # Timer to read from Arduino
        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz
    
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
                
                # Check if it's encoder data
                if line.startswith('E,'):
                    msg = String()
                    msg.data = line
                    self.encoder_pub.publish(msg)
                else:
                    # Log other messages from Arduino
                    self.get_logger().info(f'Arduino: {line}')
        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')
    
    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
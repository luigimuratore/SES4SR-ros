import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.get_logger().info('Arduino bridge started on /dev/ttyACM0')

    def cmd_callback(self, msg):
        lin = msg.linear.x
        ang = msg.angular.z
        line = f"{lin:.3f} {ang:.3f}\n"
        self.ser.write(line.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
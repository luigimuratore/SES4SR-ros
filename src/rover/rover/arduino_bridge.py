import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        
        # Connect to Arduino with retry
        self.ser = None
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=timeout)
                time.sleep(2)  # Wait for Arduino reset
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f'Arduino bridge started on {port} at {baud_rate} baud')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to connect to Arduino (attempt {attempt+1}/{max_retries}): {e}')
                if attempt < max_retries - 1:
                    time.sleep(2)
                else:
                    self.get_logger().error('Could not connect to Arduino. Exiting.')
                    raise
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Watchdog timer - stop motors if no command received for 1 second
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timeout = 1.0  # seconds
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info('Arduino bridge ready, listening on /cmd_vel')

    def cmd_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial port not open')
            return
        
        lin = msg.linear.x
        ang = msg.angular.z
        
        # Format: "linear_x angular_z\n"
        line = f"{lin:.3f} {ang:.3f}\n"
        
        try:
            self.ser.write(line.encode('utf-8'))
            self.last_cmd_time = self.get_clock().now()
            self.get_logger().debug(f'Sent to Arduino: {line.strip()}')
            
            # Read any responses from Arduino (optional)
            while self.ser.in_waiting:
                response = self.ser.readline().decode('utf-8', errors='replace').strip()
                if response:
                    self.get_logger().info(f'Arduino: {response}')
                    
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')

    def watchdog_callback(self):
        """Stop motors if no command received recently"""
        now = self.get_clock().now()
        time_since_last_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.watchdog_timeout:
            # Send stop command
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"0.000 0.000\n")
                    self.get_logger().warn('Watchdog: No recent commands, stopping motors')
                    self.last_cmd_time = now  # Reset to avoid spamming
                except Exception as e:
                    self.get_logger().error(f'Watchdog error: {e}')

    def destroy_node(self):
        # Stop motors on shutdown
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"0.000 0.000\n")
                time.sleep(0.1)
                self.ser.close()
                self.get_logger().info('Arduino bridge closed')
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
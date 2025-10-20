import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControllerReset(Node):
    def __init__(self):
        super().__init__('controller_reset')
        
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for velocity commands
        self.subscription_reset = self.create_subscription(Bool,'/reset',self.reset_callback,10) # Subscription for reset signal
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Control variables 
        self.phase = 0  # 0: +X, 1: +Y, 2: -X, 3: -Y
        self.N = 1
        self.count = 0
        
        self.get_logger().info('Controller Reset Node initialized with N=1, phase=0')

    def reset_callback(self, msg: Bool): #Reset controller when receiving True on /reset topic
        if msg.data:
            self.get_logger().warn('🔄 RESET RECEIVED - Restarting control sequence with N=1, phase=0')
            
            # Stop the robot first
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            self.publisher_.publish(stop_msg)
            
            # Reset control variables
            self.phase = 0
            self.N = 1
            self.count = 0

    def timer_callback(self):
        msg = Twist()
        
        # Determine direction based on phase 
        if self.phase == 0:
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            direction = "X-axis"
        elif self.phase == 1:
            msg.linear.x = 0.0
            msg.linear.y = 1.0
            direction = "Y-axis"
        elif self.phase == 2:
            msg.linear.x = -1.0
            msg.linear.y = 0.0
            direction = "opposite X-axis"
        elif self.phase == 3:
            msg.linear.x = 0.0
            msg.linear.y = -1.0
            direction = "opposite Y-axis"

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: linear.x={msg.linear.x}, linear.y={msg.linear.y}, '
            f'phase={self.phase+1} ({direction}), N={self.N}'
        )

        self.count += 1
        if self.count >= self.N:
            self.count = 0
            self.phase += 1
            if self.phase > 3:
                self.phase = 0
                self.N += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerReset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
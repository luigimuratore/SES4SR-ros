import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for velocity commands
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
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
            f'Published: linear.x={msg.linear.x}, linear.y={msg.linear.y}, phase={self.phase+1} ({direction}), N={self.N}'
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
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
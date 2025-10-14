import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.x = 0.0  
        self.y = 0.0  
        self.period = 1.0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: linear.x={msg.linear.x}, linear.y={msg.linear.y}')
        self.print_pose(msg)


    def print_pose(self, msg):
        # Update position estimate
        self.x += msg.linear.x * self.period
        self.y += msg.linear.y * self.period

        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0

        self.publisher_.publish(pose_msg)
        
        self.get_logger().info(f'Published Pose: x={pose_msg.position.x}, y={pose_msg.position.y}')
        self.get_logger().info(f'-------------------------------------------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

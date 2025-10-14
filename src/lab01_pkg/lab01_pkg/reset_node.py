import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool 

class reset_node(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(
            Pose,
            '/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_reset = self.create_publisher(Bool, '/reset', 10)

    def listener_callback(self, msg: Pose):
        current_x = msg.position.x
        current_y = msg.position.y
        distance = math.sqrt(current_x**2 + current_y**2)
        is_far = distance > 6
        
        reset_msg = Bool()
        reset_msg.data = is_far
        
        self.publisher_reset.publish(reset_msg)
        self.get_logger().info(
            f'Received Pose: X={current_x:.2f}, Y={current_y:.2f}. '
            f'Distance from origin: {distance:.2f} m. '
            f'Is distance > 6m: {is_far}'
        )
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = reset_node()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
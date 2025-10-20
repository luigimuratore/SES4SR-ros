import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool 

class Reset_Node(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(Pose,'/pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.publisher_reset = self.create_publisher(Bool, '/reset', 10)
        self.last_reset_state = None  # Track previous state

    def listener_callback(self, msg: Pose):
        current_x = msg.position.x
        current_y = msg.position.y
        distance = math.sqrt(current_x**2 + current_y**2)
        is_far = distance > 6.0 #When the distance from the origin of the reference frame is larger than 6.0 m
        
        # Only publish when state changes
        if is_far != self.last_reset_state:
            reset_msg = Bool()
            reset_msg.data = is_far
            self.publisher_reset.publish(reset_msg)
            self.get_logger().warn(f'Reset signal: {is_far}, Distance: {distance:.2f} m')
            self.last_reset_state = is_far
        else:
            self.get_logger().debug(f'Distance: {distance:.2f} m')



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Reset_Node()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
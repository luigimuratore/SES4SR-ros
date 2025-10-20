import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class LocalizationReset(Node):
    def __init__(self):
        super().__init__('localization_reset')
        
        # Initial position 
        self.x = 0.0
        self.y = 0.0
        
        # Sampling period (Delta T) in seconds: 1 s
        self.DT = 1.0
        
        
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10) # Publisher for position
        self.subscription_cmd = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10) # Subscription for velocity
        self.subscription_reset = self.create_subscription(Bool,'/reset',self.reset_callback,10) # Subscription for reset signal
        
        self.get_logger().info("Localization Reset node started. Initial position: (0.0, 0.0)")

    def reset_callback(self, msg: Bool): #Reset position to origin when receiving True on /reset topic
        if msg.data:
            self.get_logger().warn('🔄 RESET RECEIVED - Position reset to origin (0.0, 0.0)')
            self.x = 0.0
            self.y = 0.0

    def listener_callback(self, msg: Twist):
        # linear velocity
        v_x = msg.linear.x
        v_y = msg.linear.y
        
        # new position = old position + speed * DT
        self.x += v_x * self.DT
        self.y += v_y * self.DT
        
        pose_msg = Pose() # publish the pose
        
        # position
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0

        # no rotation -> (x=0, y=0, z=0, w=1)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0 # real part

        self.publisher_pose.publish(pose_msg)

        # Log with logger
        self.get_logger().info(
            f'Received: vx={v_x:.1f}, vy={v_y:.1f}. '
            f'New Pose published: X={self.x:.2f}, Y={self.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationReset()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
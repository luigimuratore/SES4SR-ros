import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from landmark_msgs.msg import Landmark, LandmarkArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
from angles import normalize_angle


class LandmarkSimulator(Node):

    def __init__(self):
        super().__init__("landmark_simulator")

        # dynamically reconfigurable parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("field_of_view_deg", 80.0),
                ("max_range", 2.5),
                ("min_range", 0.3),
                ("range_stddev", 0.01),
                ("bearing_stddev_deg", 0.01),
            ],
        )

        # Load landmarks from file
        default_landmarks_file = os.path.join(
            get_package_share_directory("turtlebot3_perception"), "config", "landmarks.yaml"
        )
        landmarks_file = (
            self.declare_parameter("landmarks_file", default_landmarks_file).get_parameter_value().string_value
        )
        with open(landmarks_file, "r") as file:
            landmarks = yaml.safe_load(file)
            self.landmarks = landmarks["landmarks"]
            if any(len(self.landmarks["id"]) != len(lst) for lst in self.landmarks.values()):
                self.get_logger().error("All lists in the landmarks file must have the same length")
                raise ValueError()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.landmark_array_pub = self.create_publisher(LandmarkArray, "landmarks", 10)
        self.create_subscription(Odometry, "ground_truth", self.pose_update, 10)
        self.current_pose = None
        freq = self.declare_parameter("frequency_hz", 10.0).get_parameter_value().double_value
        self.create_timer(1 / freq, self.timer_callback)

    def pose_update(self, msg: Odometry):
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return

        max_range = self.get_parameter("max_range").get_parameter_value().double_value
        min_range = self.get_parameter("min_range").get_parameter_value().double_value
        fov = math.radians(self.get_parameter("field_of_view_deg").get_parameter_value().double_value)
        range_stddev = self.get_parameter("range_stddev").get_parameter_value().double_value
        bearing_stddev = math.radians(self.get_parameter("bearing_stddev_deg").get_parameter_value().double_value)

        # Initialize containers to store landmark informations
        transforms = []
        landmarks_msg = LandmarkArray()
        landmarks_msg.header.stamp = self.get_clock().now().to_msg()
        landmarks_msg.header.frame_id = self.current_pose.header.frame_id

        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        for id, x, y in zip(self.landmarks["id"], self.landmarks["x"], self.landmarks["y"]):

            # Is the landmark within the field of view? if not contiue to the next landmark
            range = math.sqrt((robot_x - x) ** 2 + (robot_y - y) ** 2) + np.random.normal(0, range_stddev)
            if range > max_range or range < min_range:
                continue
            quat = self.current_pose.pose.pose.orientation
            yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            bearing = normalize_angle(math.atan2(y - robot_y, x - robot_x) - yaw + np.random.normal(0, bearing_stddev))
            if abs(bearing) > fov / 2:
                continue

            landmark_msg = Landmark()
            landmark_msg.id = id
            landmark_msg.range = range
            landmark_msg.bearing = bearing
            landmarks_msg.landmarks.append(landmark_msg)

            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.current_pose.child_frame_id
            tf.child_frame_id = f"landmark_{id}"
            tf.transform.translation.x = landmark_msg.range * math.cos(landmark_msg.bearing)
            tf.transform.translation.y = landmark_msg.range * math.sin(landmark_msg.bearing)
            tf.transform.rotation.w = 1.0
            transforms.append(tf)

        self.landmark_array_pub.publish(landmarks_msg)
        self.tf_broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = LandmarkSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # cleans up pub-subs, etc
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarDebug(Node):
    def __init__(self):
        super().__init__('lidar_debug')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.get_logger().info('LiDAR Debug Node started')
    
    def scan_callback(self, msg):
        """Analyze LiDAR data to find where obstacles are"""
        self.get_logger().info(
            f'\n========== LiDAR Analysis ==========\n'
            f'Total points: {len(msg.ranges)}\n'
            f'Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°\n'
            f'Angle increment: {math.degrees(msg.angle_increment):.2f}°'
        )
        
        # Find closest reading and its angle
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max):
                angle = msg.angle_min + i * msg.angle_increment
                valid_ranges.append((i, angle, r))
        
        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR readings!')
            return
        
        # Sort by distance
        valid_ranges.sort(key=lambda x: x[2])
        
        self.get_logger().info('\n===== 10 Closest Readings =====')
        for i, (idx, angle, distance) in enumerate(valid_ranges[:10]):
            self.get_logger().info(
                f'{i+1}. Index {idx:3d} | Angle: {math.degrees(angle):6.1f}° | Distance: {distance:.2f}m'
            )
        
        # Analyze sectors
        front_ranges = []
        left_ranges = []
        right_ranges = []
        back_ranges = []
        
        for idx, angle, r in valid_ranges:
            angle_deg = math.degrees(angle)
            if -30 <= angle_deg <= 30:
                front_ranges.append(r)
            elif 30 < angle_deg <= 150:
                left_ranges.append(r)
            elif -150 <= angle_deg < -30:
                right_ranges.append(r)
            else:
                back_ranges.append(r)
        
        self.get_logger().info(
            f'\n===== Sector Analysis =====\n'
            f'Front (-30° to +30°): {len(front_ranges)} points, min: {min(front_ranges) if front_ranges else "N/A"}\n'
            f'Left (+30° to +150°): {len(left_ranges)} points, min: {min(left_ranges) if left_ranges else "N/A"}\n'
            f'Right (-150° to -30°): {len(right_ranges)} points, min: {min(right_ranges) if right_ranges else "N/A"}\n'
            f'Back (rest): {len(back_ranges)} points, min: {min(back_ranges) if back_ranges else "N/A"}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np
import matplotlib.pyplot as plt

from functools import wraps
import time

from turtlebot3_perception.laserscan2lines import laserscan2lines


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        print(f'Function {func.__name__} Took {total_time:.4f} seconds, {1/total_time:.4f} Hz')
        return result
    return timeit_wrapper

@timeit
def timed_laserscan2lines(scan: LaserScan):
    return laserscan2lines(scan)


class LaserScan2LinesNode(Node):

    def __init__(self):
        super().__init__('laserscan2lines')

        self.create_subscription(LaserScan, "scan", self.callback, 2)
        plt.figure(figsize=(8, 5))
        plt.show(block=False)
        self.plot_locked = False


    def callback(self, msg: LaserScan):

        lines, line_equations = timed_laserscan2lines(msg)

        self.get_logger().info(f"Toatal lines detected {len(lines)}")
        for line_points, (slope, intercept) in zip(lines, line_equations):
            if slope == np.inf:
                self.get_logger().info(f"Detected line with {len(line_points)} points, [x = {intercept:.2f}]")
            else:
                self.get_logger().info(f"Detected line with {len(line_points)} points, [y = {slope:.2f}x + {intercept:.2f}]")

        
        # Plot the points and lines
        self.plot_lines_and_points(lines, line_equations)

    def plot_lines_and_points(self, lines, line_equations):        
        # Plot each line's points
        if self.plot_locked:
            return
        
        self.plot_locked = True
        plt.cla()
        for idx, line_points in enumerate(lines):
            plt.scatter(line_points[:, 0], line_points[:, 1], label=f'Line {idx+1} Points')

            # Plot the fitted line
            slope, intercept = line_equations[idx]
            if slope == np.inf:
                # vertical lines (x = c)
                x_vals = np.full(100, intercept)
                y_vals = np.linspace(np.min(line_points[:, 1]), np.max(line_points[:, 1]), 100)
                plt.plot(x_vals, y_vals, label=f'Line {idx+1}: x = {intercept:.2f}', linewidth=2)
            else:
                # normal lines (y = mx + c)
                x_vals = np.linspace(np.min(line_points[:, 0]), np.max(line_points[:, 0]), 100)
                y_vals = slope * x_vals + intercept
                plt.plot(x_vals, y_vals, label=f'Line {idx+1}: y = {slope:.2f}x + {intercept:.2f}', linewidth=2)
        plt.xlim(-3.5, 3.5)
        plt.ylim(-3.5, 3.5)
        plt.axis('equal')
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("Detected Lines from LaserScan Data")
        plt.legend(loc='lower left')
        plt.grid()
        plt.draw()
        plt.pause(0.005)
        self.plot_locked = False

def main():
    rclpy.init()
    node = LaserScan2LinesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # cleans up pub-subs, etc
        rclpy.try_shutdown()     

if __name__ == "__main__":
    main()
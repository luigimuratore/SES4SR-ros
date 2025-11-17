from sensor_msgs.msg import LaserScan

import numpy as np
from sklearn.linear_model import RANSACRegressor

def laserscan2lines(scan: LaserScan):
    """
    A function to detect lines from a LaserScan message using RANSAC.
    A normal line is represented by y = mx + c, where m is the slope and c is the y-intercept.
    If the line is vertical (angle >85°), the line equation is x = c, where c is the x-intercept.

    :param scan: A sensor_msgs/LaserScan message
    :return: A list of detected lines, where each line is a list of points (x, y) and the line equation (slope, intercept). If the line is vertical, the slope is infinity.
    """

    # Convert LaserScan ranges to Cartesian coordinates
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)

    # Filter out invalid ranges (e.g., values at infinity)
    valid_indices = np.isfinite(ranges) & (ranges < scan.range_max) & (ranges > scan.range_min)
    angles = angles[valid_indices]
    ranges = ranges[valid_indices]

    # Calculate x and y coordinates
    x_points = ranges * np.cos(angles)
    y_points = ranges * np.sin(angles)
    points = np.vstack((x_points, y_points)).T

    lines = []  # To store detected lines
    line_equations = []  # To store line equations

    # Detect multiple lines using iterative RANSAC
    min_points_for_line = 25  # Minimum points to define a line
    ransac = RANSACRegressor(residual_threshold=0.1)

    while len(points) > min_points_for_line: 
        with np.errstate(divide='ignore'):
            stDevX = np.std(points[:, 0]) 
            stDevY = np.std(points[:, 1])
            verticality = stDevY/stDevX

        if verticality < 1.5:
            # Fit RANSAC model to the remaining points
            ransac.fit(points[:, 0].reshape(-1, 1), points[:, 1])
            line_model = ransac.estimator_
            slope = line_model.coef_[0]   # m in y = mx + c
            intercept = line_model.intercept_  # c in y = mx + c
        elif verticality < 15:
            ransac.fit(points[:, 1].reshape(-1, 1), points[:, 0])
            line_model = ransac.estimator_
            slope = line_model.coef_[0]
            intercept = line_model.intercept_
            intercept = - intercept / slope
            slope = 1 / slope
        else:
            ransac.fit(points[:, 1].reshape(-1, 1), points[:, 0])
            line_model = ransac.estimator_
            slope = np.inf
            intercept = line_model.intercept_  # c in x = c

        # Extract inliers and outliers
        inlier_mask = ransac.inlier_mask_
        line_points = points[inlier_mask]

        # Save the detected line
        if len(line_points) >= min_points_for_line:
            lines.append(line_points)

            # Get line equation parameters
            line_equations.append((slope, intercept))
        # Remove inlier points and continue
        points = points[~inlier_mask]

    
    return lines, line_equations
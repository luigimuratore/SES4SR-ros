import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Arc

from .models.landmark_model import (
    landmark_range_bearing_sensor,
    landmark_model_prob,
    plot_sampled_poses,
    plot_landmarks,
) # import functions from models

arrow = u'$\u2191$'

def compute_jacobians(robot_pose, landmark):
    # Compute the analytical Jacobian of the landmark measurement model.
    # Inputs:
    #   - robot_pose: the robot pose [x,y,theta]
    #   - landmark: the landmark position in the map [m_x, m_y]

    x, y, theta = robot_pose[:]
    m_x, m_y = landmark[:]
    
    dx = m_x - x
    dy = m_y - y
    q = dx**2 + dy**2
    sqrt_q = math.sqrt(q)
    
    # Compute Jacobian entries
    dr_dx = -dx / sqrt_q
    dr_dy = -dy / sqrt_q
    dr_dtheta = 0.0
    
    dphi_dx = dy / q
    dphi_dy = -dx / q
    dphi_dtheta = -1.0
    
    # Assemble Jacobian matrix
    H = np.array([[dr_dx, dr_dy, dr_dtheta],
                  [dphi_dx, dphi_dy, dphi_dtheta]])
    
    return H


def main():
    # robot pose
    robot_pose = np.array([0., 0., math.pi/4])
    # landmarks position in the map
    landmarks = [
                 np.array([5., 2.]),
                 np.array([-2.5, 3.]),
                 np.array([3., 1.5]),
                 np.array([4., -1.]),
                 np.array([-2., -2.])
                 ]
    # sensor parameters
    fov = math.pi/3
    max_range = 6.0
    sigma = np.array([0.3, math.pi/24])

    # compute measurements and associated probability
    z = []
    p = []
    for i in range(len(landmarks)):
        # read sensor measurements (range, bearing)
        z_i = landmark_range_bearing_sensor(robot_pose, landmarks[i], sigma=sigma, max_range=max_range, fov=fov)
         
        if z_i is not None: # if landmark is not detected, the measurement is None
            z.append(z_i)
            # compute the probability for each measurement according to the landmark model algorithm
            p_i = landmark_model_prob(z_i, landmarks[i], robot_pose, max_range, fov, sigma)
            p.append(p_i)

    print("Probability density value:", p)
    # Plot landmarks, robot pose with sensor FOV, and detected landmarks with associated probability
    plot_landmarks(landmarks, robot_pose, z, p, fov=fov)

    
    #Sampling poses from landmark model
    if len(z) == 0:
        print("No landmarks detected!")
        return
    
    # consider only the first landmark detected
    landmark = landmarks[0]
    z = landmark_range_bearing_sensor(robot_pose, landmark, sigma)

    print(f"\n{'='*60}")
    print(f"TASK: Sampling poses from landmark model")
    print(f"{'='*60}")
    print(f"Robot pose: x={robot_pose[0]:.2f}, y={robot_pose[1]:.2f}, theta={math.degrees(robot_pose[2]):.2f}°")
    print(f"Landmark position: x={landmark[0]:.2f}, y={landmark[1]:.2f}")
    print(f"Measurement: range={z[0]:.2f}m, bearing={math.degrees(z[1]):.2f}°")
    
    # Compute and display Jacobian
    print(f"\n{'='*60}")
    print(f"Computing Jacobian H of measurement model")
    print(f"{'='*60}")
    H = compute_jacobians(robot_pose, landmark)
    print("Jacobian H (∂h/∂x) =")
    print(f"[∂r/∂x    ∂r/∂y    ∂r/∂theta  ]   [{H[0,0]:8.4f}  {H[0,1]:8.4f}  {H[0,2]:8.4f}]")
    print(f"[∂phi/∂x  ∂phi/∂y  ∂phi/∂theta] = [{H[1,0]:8.4f}  {H[1,1]:8.4f}  {H[1,2]:8.4f}]")

    # plot landmark
    plt.plot(landmark[0], landmark[1], "sk", ms=10, label='Landmark')
    plot_sampled_poses(robot_pose, z, landmark, sigma)
    
    plt.close('all')

if __name__ == "__main__":
    main()
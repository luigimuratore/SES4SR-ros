import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from  matplotlib.patches import Arc
from utils import compute_p_hit_dist
arrow = u'$\u2191$'

def landmark_range_bearing_sensor(robot_pose, landmark, sigma, max_range=6.0, fov=math.pi/2):
    """""
    Simulate the detection of a landmark with a virtual sensor able to estimate range and bearing
    """""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]

    r_ = math.dist([x, y], [m_x, m_y]) + np.random.normal(0., sigma[0])
    phi_ = math.atan2(m_y - y, m_x - x) - theta + np.random.normal(0., sigma[1])

    # filter z for a more realistic sensor simulation (add a max range distance and a FOV)
    if r_ > max_range or abs(phi_) > fov / 2:
        return None

    return [r_, phi_]

def landmark_model_prob(z, landmark, robot_pose, max_range, fov, sigma):
    """""
    Landmark sensor model algorithm:
    Inputs:
      - z: the measurements features (range and bearing of the landmark from the sensor) [r, phi]
      - landmark: the landmark position in the map [m_x, m_y]
      - x: the robot pose [x,y,theta]
    Outputs:
     - p: the probability p(z|x,m) to obtain the measurement z from the state x
        according to the estimated range and bearing
    """""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]
    sigma_r, sigma_phi = sigma[:]

    r_hat = math.dist([x, y], [m_x, m_y])
    phi_hat = math.atan2(m_y - y, m_x - x) - theta
    p = compute_p_hit_dist(z[0] - r_hat, max_range, sigma_r) * compute_p_hit_dist(z[1] - phi_hat, fov/2, sigma_phi)

    return p

def landmark_model_sample_pose(z, landmark, sigma):
    """""
    Sample a robot pose from the landmark model
    Inputs:
        - z: the measurements features (range and bearing of the landmark from the sensor) [r, phi]
        - landmark: the landmark position in the map [m_x, m_y]
        - sigma: the standard deviation of the measurement noise [sigma_r, sigma_phi]
    Outputs:
        - x': the sampled robot pose [x', y', theta']
    """""
    m_x, m_y = landmark[:]
    sigma_r, sigma_phi = sigma[:]

    gamma_hat = np.random.uniform(0, 2*math.pi)
    r_hat = z[0] + np.random.normal(0, sigma_r)
    phi_hat = z[1] + np.random.normal(0, sigma_phi)

    x_ = m_x + r_hat * math.cos(gamma_hat)
    y_ = m_y + r_hat * math.sin(gamma_hat)
    theta_ = gamma_hat - math.pi - phi_hat

    return np.array([x_, y_, theta_])


def plot_sampled_poses(robot_pose, z, landmark, sigma, n_samples=1000):
    """
    Plot sampled poses from the landmark model with statistics
    """
    samples = []
    
    # Generate samples
    for i in range(n_samples):
        x_prime = landmark_model_sample_pose(z, landmark, sigma)
        samples.append(x_prime)
        
        # plot robot pose (make it less dense for visualization)
        if i % 10 == 0:  # plot every 10th sample
            rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
            rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(x_prime[2])-90)
            plt.scatter(x_prime[0], x_prime[1], marker=rotated_marker, s=80, 
                       facecolors='none', edgecolors='b', alpha=0.3)
    
    # Convert to numpy array
    samples = np.array(samples)
    
    # Compute statistics
    mean_pose = np.mean(samples, axis=0)
    std_pose = np.std(samples, axis=0)
    
    print(f"\n{'='*60}")
    print(f"Sampling Statistics (N={n_samples})")
    print(f"{'='*60}")
    print(f"Mean pose: x={mean_pose[0]:.3f}±{std_pose[0]:.3f}, "
          f"y={mean_pose[1]:.3f}±{std_pose[1]:.3f}, "
          f"theta={math.degrees(mean_pose[2]):.1f}±{math.degrees(std_pose[2]):.1f}°")
    print(f"True pose: x={robot_pose[0]:.3f}, y={robot_pose[1]:.3f}, "
          f"theta={math.degrees(robot_pose[2]):.1f}°")
    
    # Plot mean pose
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(mean_pose[2])-90)
    plt.scatter(mean_pose[0], mean_pose[1], marker=rotated_marker, s=200, 
               facecolors='none', edgecolors='g', linewidths=2, label='Mean sampled pose')
    
    # Plot real pose
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose[2])-90)
    plt.scatter(robot_pose[0], robot_pose[1], marker=rotated_marker, s=200, 
               facecolors='none', edgecolors='r', linewidths=2, label='True pose')
    
    # Plot covariance ellipse
    plot_covariance_ellipse(mean_pose[:2], samples[:, :2])
    
    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title(f"Landmark Model Pose Sampling (N={n_samples})")
    plt.legend()
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    # plt.savefig("landmark_model_sampling.pdf")
    plt.show()


def plot_landmarks(landmarks, robot_pose, z, p_z, max_range=6.0, fov=math.pi/4):
    """""
    Plot landmarks, robot pose with sensor FOV, and detected landmarks with associated probability
    """""
    x, y, theta = robot_pose[:]

    start_angle = theta + fov/2
    end_angle = theta - fov/2

    plt.figure()
    ax = plt.gca()
    # plot robot pose
    # find the virtual end point for orientation
    endx = x + 0.5 * math.cos(theta)
    endy = y + 0.5 * math.sin(theta)
    plt.plot(x, y, 'or', ms=10)
    plt.plot([x, endx], [y, endy], linewidth = '2', color='r')

    # plot FOV
    # get ray target coordinates
    fov_x_left = x + math.cos(start_angle) * max_range
    fov_y_left = y + math.sin(start_angle) * max_range
    fov_x_right = x + math.cos(end_angle) * max_range
    fov_y_right = y + math.sin(end_angle) * max_range

    plt.plot([x, fov_x_left], [y, fov_y_left], linewidth = '1', color='b')
    plt.plot([x, fov_x_right], [y, fov_y_right], linewidth = '1', color='b')

    R = max_range
    a, b = 2*R, 2*R
    arc = Arc((x, y), a, b,
                 theta1=math.degrees(end_angle), theta2=math.degrees(start_angle), color='b', lw=1.2)
    ax.add_patch(arc)

    # plot landmarks
    for i, lm in enumerate(landmarks):
        plt.plot(lm[0], lm[1], "sk", ms=10, alpha=0.7)

    # plot perceived landmarks position and associated probability (color scale)
    lm_z = np.zeros((len(z), 2))
    for i in range(len(z)):
        # draw endpoint with probability from Likelihood Fields
        lx = x + z[i][0] * math.cos(z[i][1]+theta)
        ly = y + z[i][0] * math.sin(z[i][1]+theta)
        lm_z[i, :] = lx, ly
    
    col = np.array(p_z)
    plt.scatter(lm_z[:,0], lm_z[:,1], s=60, c=col, cmap='viridis')
    plt.colorbar()

    plt.show()
    plt.close('all')


def compute_measurement_jacobian(robot_pose, landmark):
    """
    Compute the Jacobian H of the landmark measurement model with respect to the robot state
    
    The measurement model is:
        h(x) = [r, phi]^T
        where:
            r = sqrt((m_x - x)^2 + (m_y - y)^2)
            phi = atan2(m_y - y, m_x - x) - theta
    
    Inputs:
        - robot_pose: the robot pose [x, y, theta]
        - landmark: the landmark position [m_x, m_y]
    
    Outputs:
        - H: the Jacobian matrix (2x3) of the measurement model
             H = [∂h/∂x, ∂h/∂y, ∂h/∂theta]
    """
    x, y, theta = robot_pose[:]
    m_x, m_y = landmark[:]
    
    # Compute intermediate values
    delta_x = m_x - x
    delta_y = m_y - y
    q = delta_x**2 + delta_y**2  # squared distance
    sqrt_q = math.sqrt(q)
    
    # Compute Jacobian H (2x3 matrix)
    # First row: derivative of range r w.r.t. [x, y, theta]
    # Second row: derivative of bearing phi w.r.t. [x, y, theta]
    
    H = np.array([
        [-delta_x / sqrt_q,  -delta_y / sqrt_q,  0],      # ∂r/∂x, ∂r/∂y, ∂r/∂theta
        [ delta_y / q,       -delta_x / q,       -1]       # ∂phi/∂x, ∂phi/∂y, ∂phi/∂theta
    ])
    
    return H

def verify_jacobian(robot_pose, landmark, H_analytical, epsilon=1e-5):
    """
    Verify the analytical Jacobian by comparing with numerical differentiation
    
    Inputs:
        - robot_pose: the robot pose [x, y, theta]
        - landmark: the landmark position [m_x, m_y]
        - H_analytical: the analytically computed Jacobian
        - epsilon: small perturbation for numerical differentiation
    """
    x, y, theta = robot_pose[:]
    m_x, m_y = landmark[:]
    
    # Function to compute measurement
    def h(pose):
        px, py, ptheta = pose
        dx = m_x - px
        dy = m_y - py
        r = math.sqrt(dx**2 + dy**2)
        phi = math.atan2(dy, dx) - ptheta
        return np.array([r, phi])
    
    # Compute numerical Jacobian
    H_numerical = np.zeros((2, 3))
    h0 = h(robot_pose)
    
    for i in range(3):
        pose_plus = robot_pose.copy()
        pose_plus[i] += epsilon
        h_plus = h(pose_plus)
        
        pose_minus = robot_pose.copy()
        pose_minus[i] -= epsilon
        h_minus = h(pose_minus)
        
        # Central difference
        H_numerical[:, i] = (h_plus - h_minus) / (2 * epsilon)
    
    print("Numerical Jacobian H_num =")
    print(f"[{H_numerical[0,0]:8.4f}  {H_numerical[0,1]:8.4f}  {H_numerical[0,2]:8.4f}]")
    print(f"[{H_numerical[1,0]:8.4f}  {H_numerical[1,1]:8.4f}  {H_numerical[1,2]:8.4f}]")
    
    # Compute error
    error = np.linalg.norm(H_analytical - H_numerical)
    print(f"\nJacobian verification error: {error:.6e}")
    
    if error < 1e-4:
        print("✓ Jacobian is CORRECT!")
    else:
        print("✗ Jacobian may have errors!")
    
    return H_numerical

def plot_covariance_ellipse(mean, samples, n_std=2):
    """
    Plot covariance ellipse from samples
    
    Inputs:
        - mean: mean position [x, y]
        - samples: array of sampled positions (Nx2)
        - n_std: number of standard deviations for ellipse
    """
    from matplotlib.patches import Ellipse
    
    # Compute covariance matrix
    cov = np.cov(samples.T)
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    
    # Compute ellipse parameters
    angle = math.degrees(math.atan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    width = 2 * n_std * math.sqrt(eigenvalues[0])
    height = 2 * n_std * math.sqrt(eigenvalues[1])
    
    # Create ellipse
    ellipse = Ellipse(mean, width, height, angle=angle, 
                     facecolor='yellow', edgecolor='orange', 
                     alpha=0.3, linewidth=2, label=f'{n_std}σ uncertainty')
    
    plt.gca().add_patch(ellipse)

def main():
    ##############################
    ### Landmark model example ###
    ##############################

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

    ##########################################
    ### Sampling poses from landmark model ###
    ##########################################
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
    H = compute_measurement_jacobian(robot_pose, landmark)
    print("Jacobian H (∂h/∂x) =")
    print(f"[∂r/∂x    ∂r/∂y    ∂r/∂theta  ]   [{H[0,0]:8.4f}  {H[0,1]:8.4f}  {H[0,2]:8.4f}]")
    print(f"[∂phi/∂x  ∂phi/∂y  ∂phi/∂theta] = [{H[1,0]:8.4f}  {H[1,1]:8.4f}  {H[1,2]:8.4f}]")
    
    # Verify Jacobian with numerical differentiation
    print(f"\n{'='*60}")
    print(f"Verifying Jacobian with numerical differentiation")
    print(f"{'='*60}")
    verify_jacobian(robot_pose, landmark, H)

    # plot landmark
    plt.plot(landmark[0], landmark[1], "sk", ms=10, label='Landmark')
    plot_sampled_poses(robot_pose, z, landmark, sigma)
    
    plt.close('all')

if __name__ == "__main__":
    main()
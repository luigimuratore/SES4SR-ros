import numpy as np
import matplotlib.pyplot as plt

def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = theta % (2 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta>np.pi] -= 2 * np.pi
        return theta_
    
    return theta

# Exponential Function
def exponential(x, _lambda):
    return _lambda * np.exp(-1 * _lambda * x)

# Gaussian Function
def gaussian(x, mu, sigma):
    return (1.0 / (np.sqrt(2*np.pi) * sigma)) * np.exp(-0.5 * ((x - mu) / sigma)**2)

# Normalized Gaussian pdf
def compute_p_hit_dist(dist, max_dist, sigma):
    '''
    Compute the hit probability p_hit for a given distance measurement.
    Args:
        dist: observed distance measurement
        max_dist: maximum measurable distance
        sigma: standard deviation of the Gaussian noise
    Returns:
        p_hit: normalized hit probability
    '''
    # Normalize the Gaussian over [0, max_dist]
    normalize_hit = 1e-9
    for j in range(round(max_dist)):
        normalize_hit += gaussian(j, 0., sigma)
    normalize_hit = 1. / normalize_hit

    p_hit = gaussian(dist, 0., sigma)*normalize_hit

    return p_hit

# Vectorized probabilistic beam model
def precompute_p_hit_map(distances, max_dist=None, sigma=1.0):
    """
    Vectorized probabilistic beam model.
    Supports scalar or ndarray inputs for distances (must be broadcastable).
    
    Args:
        dist:     observed distances measurement(s), ndarray or float
        z_max:    max range (scalar)
        sigma:    std dev for hit Gaussian

    Returns:
        p_hit (hit prob)
    """
    dist_flat = np.asarray(distances.ravel())

    if max_dist is None:
        max_dist = np.max(dist_flat)
    
    # --- Hit mode normalization ---
    j = np.arange(int(max_dist))
    normalize_hit = np.sum(gaussian(j[:, None], dist_flat, sigma), axis=0)
    normalize_hit = normalize_hit.reshape(distances.shape)
    normalize_hit = np.where(normalize_hit > 0, 1.0 / normalize_hit, 1.0)

    # Hit probability
    p_hit = gaussian(distances, np.zeros_like(distances), sigma) * normalize_hit
    return p_hit

# Vectorized probabilistic beam model
def evaluate_range_beam_dist_array(z, z_star, z_max, _mix_density, _sigma, _lamb_short):
    """
    Vectorized probabilistic beam model.
    Supports scalar or ndarray inputs for z and z_star (must be broadcastable).
    
    Args:
        z:        observed measurement(s), ndarray or float
        z_star:   expected measurement(s), ndarray or float
        z_max:    max range (scalar)
        _mix_density: mixture weights [z_hit, z_short, z_max, z_rand]
        _sigma:   std dev for hit Gaussian
        _lamb_short: lambda for short distribution

    Returns:
        p_hit, p_short, p_max, p_rand, p (stacked array), p_z (mixture prob)
        Shapes follow broadcast of z and z_star
    """

    z = np.asarray(z)
    z_star = np.asarray(z_star)

    # --- Hit mode normalization ---
    j = np.arange(int(z_max))
    normalize_hit = np.sum(gaussian(j[:, None], z_star.ravel(), _sigma), axis=0)
    normalize_hit = normalize_hit.reshape(z_star.shape)
    normalize_hit = np.where(normalize_hit > 0, 1.0 / normalize_hit, 1.0)

    # Hit probability
    p_hit = gaussian(z, z_star, _sigma) * normalize_hit

    # --- Short mode ---
    normalize_short = 1.0 - np.exp(-_lamb_short * z_star)
    p_short = np.where(
        z <= z_star,
        _lamb_short * np.exp(-_lamb_short * z) / (normalize_short + 1e-12),
        0.0
    )

    # --- Max mode ---
    p_max = np.where(z == z_max, 1.0, 0.0)

    # --- Random mode ---
    p_rand = np.full_like(z, 1.0 / z_max, dtype=float)

    # Stack all components: shape (..., 4)
    p = np.stack([p_hit, p_short, p_max, p_rand], axis=-1)

    # Weighted mixture probability
    p_z = np.tensordot(p, _mix_density, axes=([-1],[0]))

    return p_hit, p_short, p_max, p_rand, p_z

# Plot the distribution of z samples
def plot_sampling_dist(samples, title="Distribution of z samples", fig_name="z_star_hist.pdf"):
    '''
    Plot the distribution of z samples.
    Args:
        samples: array of z samples
        title: title of the plot
        fig_name: name of the file to save the plot
    '''
    
    n_bins = 100
    plt.hist(samples, n_bins)
    plt.title(title)
    plt.grid()
    plt.savefig(fig_name)
    plt.show()
    plt.close('all')


# Bresenham's line algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
def bresenham_v0(x0, y0, x1, y1):
    ret = []

    dx =  abs(x1-x0)
    sx = 1 if (x0<x1) else -1
    dy = -abs(y1-y0)
    sy = 1 if (y0<y1) else -1
    err = dx+dy

    while (True):
        ret.append((x0, y0))
        if (x0==x1 and y0==y1):
            break
        e2 = 2*err
        if (e2 >= dy):
            err += dy
            x0 += sx
        if (e2 <= dx):
            err += dx
            y0 += sy

    return ret


def bresenham(x0, y0, x1, y1, map):
    """""
    x0, y0: coordinate of the starting point (robot position)
    x1, y1: map coordinate of the max range point
    map: 2D occupancy grid map
    return: coordinate of the obstacle point or the map boundary point
    """""

    dx =  abs(x1 - x0)
    sx = 1 if (x0 < x1) else -1
    dy = -abs(y1 - y0)
    sy = 1 if (y0 < y1) else -1
    err = dx + dy

    while (True):
        # check if obstacle encountered or ray reach end of map
        if x0 < 0.:
            obst = 0, y0
            break
        elif x0 >= map.shape[0]: # check if map border reached
            obst = x0, y0
            break
        elif y0 < 0.:
            obst = x0, 0
            break
        elif y0 >= map.shape[1]:
            obst = x0, y0
            break
        # elif map[int(x0-1), int(y0-1)]==1 or map[int(x0), int(y0)]==1 or map[int(x0-1), int(y0)]==1 or map[int(x0), int(y0-1)]==1 or ((x0==x1) and (y0==y1)):
        #     obst = [x0, y0]
        #     print("obst", obst)
        #     break
        elif map[int(x0), int(y0)]==1 or ((x0==x1) and (y0==y1)):
            obst = [x0, y0]
            break

        e2 = 2*err
        if (e2 >= dy):
            err += dy
            x0 += sx
        if (e2 <= dx):
            err += dx
            y0 += sy
        
    return obst

def compute_measurement_jacobian(robot_pose, landmark):
    """
    Compute Jacobian of landmark measurement model (H matrix)
    From task_0_b.py
    
    Args:
        robot_pose: [x, y, theta]
        landmark: [m_x, m_y]
    
    Returns:
        H: 2x3 Jacobian matrix
    """
    x, y, theta = robot_pose
    m_x, m_y = landmark
    
    dx = m_x - x
    dy = m_y - y
    q = dx**2 + dy**2
    sqrt_q = np.sqrt(q)
    
    H = np.array([
        [-dx/sqrt_q, -dy/sqrt_q, 0.0],
        [dy/q, -dx/q, -1.0]
    ])
    
    return H


def landmark_measurement_model(robot_pose, landmark):
    """
    Compute expected measurement [range, bearing] from robot to landmark
    From task_0_b.py
    
    Args:
        robot_pose: [x, y, theta]
        landmark: [m_x, m_y]
    
    Returns:
        z_expected: [range, bearing]
    """
    x, y, theta = robot_pose
    m_x, m_y = landmark
    
    dx = m_x - x
    dy = m_y - y
    
    range_pred = np.sqrt(dx**2 + dy**2)
    bearing_pred = np.arctan2(dy, dx) - theta
    bearing_pred = normalize_angle(bearing_pred)
    
    return np.array([range_pred, bearing_pred])


def compute_motion_jacobians(dt):
    """
    Compute symbolic Jacobians for velocity motion model using SymPy
    From task_0_a.py
    
    Args:
        dt: time step (can be symbolic or numeric)
    
    Returns:
        eval_gux: function to evaluate motion model g(x, u)
        eval_Gt: function to evaluate Jacobian w.r.t. state
        eval_Vt: function to evaluate Jacobian w.r.t. control
    """
    try:
        from sympy import symbols, Matrix, sin, cos, lambdify
        import sympy
        
        # Define symbolic variables
        x_s, y_s, theta_s = symbols('x y theta')
        v_s, w_s = symbols('v w')
        dt_s = symbols('dt')
        
        # Motion model for differential drive
        # x' = x + v*cos(theta)*dt
        # y' = y + v*sin(theta)*dt
        # theta' = theta + w*dt
        
        gux = Matrix([
            [x_s + v_s * cos(theta_s) * dt_s],
            [y_s + v_s * sin(theta_s) * dt_s],
            [theta_s + w_s * dt_s]
        ])
        
        # Lambdify for numerical evaluation
        eval_gux = lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), gux, 'numpy')
        
        # Jacobian w.r.t. state [x, y, theta]
        Gt = gux.jacobian(Matrix([x_s, y_s, theta_s]))
        eval_Gt = lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), Gt, 'numpy')
        
        # Jacobian w.r.t. control [v, w]
        Vt = gux.jacobian(Matrix([v_s, w_s]))
        eval_Vt = lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), Vt, 'numpy')
        
        return eval_gux, eval_Gt, eval_Vt
        
    except ImportError:
        print("Warning: SymPy not available, returning None")
        return None, None, None


def sample_velocity_motion_model(x, u, a, dt):
    """
    Sample velocity motion model with noise
    From task_0_a.py
    
    Args:
        x: pose of the robot before moving [x, y, theta]
        u: velocity reading [v, w]
        a: noise parameters [a1, a2, a3, a4, a5, a6]
        dt: time interval
    
    Returns:
        x_new: new pose [x, y, theta] after motion
    """
    # Add noise to velocity commands
    v_hat = u[0] + np.random.normal(0, np.sqrt(a[0]*u[0]**2 + a[1]*u[1]**2))
    w_hat = u[1] + np.random.normal(0, np.sqrt(a[2]*u[0]**2 + a[3]*u[1]**2))
    gamma_hat = np.random.normal(0, np.sqrt(a[4]*u[0]**2 + a[5]*u[1]**2))
    
    # Apply motion model
    x_new = x[0] + v_hat * np.cos(x[2]) * dt
    y_new = x[1] + v_hat * np.sin(x[2]) * dt
    theta_new = x[2] + w_hat * dt + gamma_hat
    theta_new = normalize_angle(theta_new)
    
    return np.array([x_new, y_new, theta_new])


def create_motion_model_functions():
    """
    Create motion model functions for EKF (non-symbolic version)
    Simpler alternative to compute_motion_jacobians
    
    Returns:
        eval_gux: motion model function
        eval_Gt: Jacobian w.r.t. state
        eval_Vt: Jacobian w.r.t. control
    """
    
    def eval_gux(x, y, theta, v, omega, dt):
        """Motion model for differential drive"""
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = normalize_angle(theta + omega * dt)
        return np.array([x_new, y_new, theta_new])
    
    def eval_Gt(x, y, theta, v, omega, dt):
        """Jacobian w.r.t. state"""
        return np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])
    
    def eval_Vt(x, y, theta, v, omega, dt):
        """Jacobian w.r.t. control"""
        return np.array([
            [np.cos(theta) * dt, 0],
            [np.sin(theta) * dt, 0],
            [0, dt]
        ])
    
    return eval_gux, eval_Gt, eval_Vt

import numpy as np
import math


class Differential_drive_robot():
    def __init__(self, 
                init_pose,
                max_linear_acc = 0.8,
                max_ang_acc = 100 * math.pi /180,
                max_lin_vel = 1.0, # m/s
                min_lin_vel = 0.0, # m/s
                max_ang_vel = 3.0, # rad/s 
                min_ang_vel = -3.0, # rad/s 
                radius = 0.3, # radius for circular robot
                ):
        
        #initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        
        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # size
        self.radius = radius # circular shape

        # trajectory initialization
        self.trajectory = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0]).reshape(1, -1)

    def update_state(self, u, dt):
        """
        Compute next pose of the robot according to differential drive kinematics rule (platform level equation).
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)

        self.vel = u

        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = self.pose[2] + self.vel[1] * dt
        self.pose = np.array([next_x, next_y, next_th])

        traj_state = np.array([next_x, next_y, next_th, self.vel[0], self.vel[1]]).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)

        return self.pose

def calc_nearest_obs(robot_pose, obstacles, obstacle_max_dist=3):
    """
    Filter obstacles: find the ones in the local map considered for obstacle avoidance.
    """
    nearest_obs = []
    
    for obs in obstacles:
        temp_dist_to_obs = np.linalg.norm(robot_pose[0:2]-obs)

        if temp_dist_to_obs < obstacle_max_dist :
            nearest_obs.append(obs)

    return np.array(nearest_obs)

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

def normalize(arr: np.ndarray):
    """ normalize array of values """
    if np.isclose(np.max(arr) - np.min(arr), 0.0):
        return np.zeros_like(arr)
    else:
        return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))

def sigmoid(x: np.ndarray):
  """ compute sigmoid smoothing activation of a given array of values"""
  return 1/(1 + np.exp(-x)) 
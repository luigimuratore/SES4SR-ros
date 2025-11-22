import numpy as np

# import sympy
# from sympy import symbols, Matrix
import math


def normalize(arr: np.ndarray):
    """normalize vector for plots"""
    return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))


def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = np.remainder(theta, 2.0 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2.0 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta > np.pi] -= 2.0 * np.pi
        return theta_

    return theta


class RobotStates:
    """
    Class to store robot states (x, y, yaw, v, t)
    """

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.pind = []
        self.a = []

    def append(self, t, pind, state, a):
        """
        Append a state to the list of states

        Parameters
        ----------
        t : float
            time
        pind : int
            index of the waypoint
        state : object
            state of the robot
        a : float
            acceleration
        """
        self.x.append(state.pose.flatten()[0])
        self.y.append(state.pose.flatten()[1])
        self.yaw.append(state.pose.flatten()[2])
        self.v.append(state.vel.flatten()[0])
        self.a.append(a)
        self.t.append(t)
        self.pind.append(pind)

    def __len__(self):
        return len(self.x)


def interpolate_waypoints(waypoints, resolution=0.01):
    """
    Interpolate the waypoints to add more points along the path

    Args:
    waypoints: array of waypoints
    resolution: distance between two interpolated points

    Return:
    interpolated_waypoints: array of interpolated waypoints
    """
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = np.linalg.norm(p2 - p1)
        n_points = int(dist / resolution)
        x = np.linspace(p1[0], p2[0], n_points)
        y = np.linspace(p1[1], p2[1], n_points)
        interpolated_waypoints += list(zip(x, y))
    return np.array(interpolated_waypoints)


def proportional_control(v_target, v_current, kp=3.0):
    """
    Compute the control input using proportional control law
    a = kp * (v_target - v_current)

    Args:
    v_target: target velocity
    v_current: current velocity

    Return:
    a: control input computed using proportional control
    """
    a = kp * (v_target - v_current)
    return a


class DifferentialDriveRobot:
    def __init__(
        self,
        init_pose,
        max_linear_acc=0.5,
        max_ang_acc=50 * math.pi / 180,
        max_lin_vel=0.22,  # m/s
        min_lin_vel=0.0,  # m/s
        max_ang_vel=1.0,  # rad/s
        min_ang_vel=-1.0,  # rad/s
    ):

        # initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])

        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # trajectory initialization
        self.trajectory = np.array(
            [init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0], dtype=np.float64
        ).reshape(1, -1)

    def linear_velocity_update(self, a, dt):
        """
        Update the state of the robot using the control input u

        Modified variables:
        self.u: updated input
        """
        # Update the state evauating the motion model
        if a is list:
            a = np.array(a)
        # Saturate the velocities with np.clip(value, min, max)
        a = np.clip(a, -self.max_linear_acc, self.max_linear_acc)
        self.vel[0] = self.vel[0] + a * dt  # evaluate the motion model
        self.vel[0] = np.clip(self.vel[0], -self.min_lin_vel, self.max_lin_vel)
        return self.vel[0]

    def update_state(self, u, dt):
        """
        Compute next pose of the robot according to differential drive kinematics
        rule (platform level equation).
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)
        self.vel[0] = self.linear_velocity_update(u[0], dt)
        self.vel[1] = np.clip(u[1], self.min_ang_vel, self.max_ang_vel)

        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = normalize_angle(self.pose[2] + self.vel[1] * dt)
        self.pose = np.array([next_x, next_y, next_th])

        traj_state = np.array(
            [next_x, next_y, next_th, self.vel[0], self.vel[1]], dtype=np.float64
        ).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)
        return self.pose

    def reset(self, init_pose):
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        self.trajectory = np.array(
            [init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0], dtype=np.float64
        ).reshape(1, -1)

    @property
    def v(self):
        return self.vel[0]

    @property
    def w(self):
        return self.vel[1]

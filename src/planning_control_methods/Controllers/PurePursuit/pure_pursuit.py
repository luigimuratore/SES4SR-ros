import numpy as np
import math


class PurePursuitController:
    def __init__(self, robot, path, pind, Lt, vt, *args, **kwargs):
        """
        Initialize the controller using the robot and the path to track

        Args:
        robot: robot object
        path: path to track
        pind: index of the path to track
        Lt: look ahead distance
        """

        self.robot = robot  # robot object
        self.path = path  # path to track
        self.pind = pind  # index of the path to track
        self.Lt = Lt  # look ahead distance
        self.L_stop = self.Lt  # distance to stop the robot
        self._vt = vt  # desired velocity
        self.vt = vt  # target velocity

    def lookahead_point(self):
        """
        Find the nearest point on the path to the robot

        Return:
        index: index of the nearest point on the path to the robot
        goal_point: nearest point on the path to the robot
        """
        index = self.pind
        while (index + 1) < len(self.path):
            distance = np.linalg.norm(self.path[index] - self.robot.pose[:2].T)
            if distance > self.Lt:
                break
            index += 1
        if self.pind <= index:
            self.pind = index
        goal_point = self.path[self.pind]
        return index, goal_point

    def target_velocity(self):
        """
        Compute the target velocity for the robot
        Hint: check the arrive to the final goal for stopping the robot

        Return:
        vt: target velocity
        """
        dist_goal = np.linalg.norm(self.robot.pose[:2].T - self.path[-1])
        # Check if the robot is close to the final goal
        if self.pind > (len(self.path) - 2) and dist_goal < self.L_stop:
            # Stop the robot
            self.vt = 0.0
        return self.vt

    def angular_velocity(self):
        """
        Compute the angular velocity for the robot

        Modified variables:
        self.kappa: curvature of the path at the goal point
        Return:
        w: angular velocity
        """
        index, goal_point = self.lookahead_point()
        ty_r = -math.sin(self.robot.pose[2]) * (
            goal_point[0] - self.robot.pose[0]
        ) + math.cos(self.robot.pose[2]) * (goal_point[1] - self.robot.pose[1])
        self.kappa = 2 * ty_r / (self.Lt**2)
        w = self.kappa * self.target_velocity()  # the angular velocity
        return w

    def reset(self, path, pose):
        """
        Reset the controller variables

        Args:
        path: path to track
        pose: initial pose of the robot
        """
        self.path = path
        self.pind = 0
        self.robot.reset(pose)
        self.vt = self._vt

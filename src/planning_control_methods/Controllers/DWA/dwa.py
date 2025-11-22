import numpy as np
from utils import Differential_drive_robot, normalize_angle, normalize, calc_nearest_obs

# DWA
class DWA():
    def __init__(self,
                 dt = 0.1,
                 sim_time = 2.0,
                 time_granularity = 0.1,
                 v_samples = 10,
                 w_samples = 20,
                 goal_dist_tol = 0.3,
                 collision_tol = 0.3,
                 weight_angle = 0.04,
                 weight_vel = 0.2,
                 weight_obs = 0.1,
                 obstacles_map = None, # A list of obstacles poses in the map # TO DO: accept a gridmap and use local map
                 **kwargs
                 ):

        self.dt = dt
        self.sim_step = round(sim_time / time_granularity)
        self.robot = Differential_drive_robot(**kwargs)
        self.obstacles = obstacles_map 
        self.goal_dist_tol = goal_dist_tol
        self.collision_tol = collision_tol

        self.v_samples = v_samples
        self.w_samples = w_samples

        # define weight
        self.weight_angle = weight_angle
        self.weight_vel = weight_vel
        self.weight_obs = weight_obs

        self.obstacle_max_dist = 3 # like a local costmap size
        self.max_num_steps = 300
        self.feedback_rate = 50
        self.obst_tolerance = 0.5

    def go_to_pose(self, goal_pose):
        """
        Assign a target goal to the DWA controller.
        Compute commands until goal is reached.
        Provide intermediate feedback on the navigation task.
        """
        if goal_pose is list:
            goal_pose = np.array(goal_pose)

        success = False
        dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)

        print("Initial distance to goal: ", dist_to_goal)
        print("Initial Robot pose: ", self.robot.pose)

        steps = 1
        while steps <= self.max_num_steps:
            # 1. Check if Goal reached
            dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)
            if dist_to_goal < self.goal_dist_tol:
                success = True
                print("Goal reached!")
                break

            # 2. Get new observations for obstacles is needed
            # no lidar used here

            # 3. Compute command for the robot with DWA controller
            u = self.compute_cmd(goal_pose, self.robot.pose, self.obstacles)

            # 4. Send the command or update the robot pose
            pose = self.robot.update_state(u, self.dt)
            
            # 5. Provide intermediate task feedback
            if steps % self.feedback_rate == 0:
                dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)
                print("Current distance to goal ", dist_to_goal, " at step ", steps)
                print("Current Robot pose: ", pose)

            steps += 1

        if steps > self.max_num_steps:
            print("Timeout! Goal not reached.")

        return success, self.robot.trajectory
    
    def compute_cmd(self, goal_pose, robot_state, obstacles):
        """
        Compute the next velocity command u=(v,w) according to the DWA algorithm.
        The velocity leading to the highest scored trajectory is selected.
        """
        # create path
        paths, velocities = self.get_trajectories(robot_state)

        # evaluate path
        opt_idx = self.evaluate_paths(paths, velocities, goal_pose, robot_state, obstacles)
        u = velocities[opt_idx]
        return u

    def get_trajectories(self, robot_pose): 
        """
        Get all the reachable velocities inside the dynamic window
        Simulate all trajectory with velocity values u=(v,w)
        Return: 
        - simulated paths : np.ndarray,  shape: (n_paths, sim_step, state_dim)
        - velocities : np.ndarray, shape (n_paths, 2)
        """
        # calculate reachable range of velocity and angular velocity in the dynamic window
        min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel = self.compute_dynamic_window(self.robot.vel)
        
        v_values = np.linspace(min_lin_vel, max_lin_vel, self.v_samples)
        w_values = np.linspace(min_ang_vel, max_ang_vel, self.w_samples)

        # list of all paths and velocities
        n_paths = w_values.shape[0]*v_values.shape[0]
        sim_paths = np.zeros((n_paths, self.sim_step, robot_pose.shape[0]))
        velocities = np.zeros((n_paths, 2))

        # evaluate all velocities and angular velocities combinations    
        vv, ww = np.meshgrid(v_values, w_values)
        velocities = np.dstack([vv,ww]).reshape(n_paths, 2)
        sim_paths = self.simulate_paths(n_paths, robot_pose, velocities)

        return sim_paths, velocities
    
    def simulate_paths(self, n_paths, pose, u):
        """
        Simulate trajectory at constant velocity u=(v,w)
        """
        sim_paths = np.zeros((n_paths, self.sim_step, pose.shape[0]))
        sim_paths[:, 0] = pose.copy()

        for i in range(1, self.sim_step):
            sim_paths[:, i, 0] = sim_paths[:, i - 1, 0] + u[:, 0] * np.cos(sim_paths[:, i - 1, 2]) * self.dt
            sim_paths[:, i, 1] = sim_paths[:, i - 1, 1] + u[:, 0] * np.sin(sim_paths[:, i - 1, 2]) * self.dt
            sim_paths[:, i, 2] = sim_paths[:, i - 1, 2] + u[:, 1] * self.dt

        return sim_paths

    def compute_dynamic_window(self, robot_vel): 
        """
        Calculate the dynamic window composed of reachable linear velocity and angular velocity according to robot's kinematic limits.
        """
        # linear velocity
        min_vel = robot_vel[0] - self.dt * self.robot.max_linear_acc
        max_vel = robot_vel[0] + self.dt * self.robot.max_linear_acc
        # minimum
        if min_vel < self.robot.min_lin_vel:
            min_vel = self.robot.min_lin_vel
        # maximum
        if max_vel > self.robot.max_lin_vel:
            max_vel = self.robot.max_lin_vel

        # angular velocity
        min_ang_vel = robot_vel[1] - self.dt * self.robot.max_ang_acc
        max_ang_vel = robot_vel[1] + self.dt * self.robot.max_ang_acc
        # minimum
        if min_ang_vel < self.robot.min_ang_vel:
            min_ang_vel = self.robot.min_ang_vel
        # maximum
        if max_ang_vel > self.robot.max_ang_vel:
            max_ang_vel = self.robot.max_ang_vel

        return min_vel, max_vel, min_ang_vel, max_ang_vel


    def evaluate_paths(self, paths, velocities, goal_pose, robot_pose, obstacles):
        """
        Evaluate the simulated paths using the objective function.
        J = w_h * heading + w_v * vel + w_o * obst_dist
        """
        # detect nearest obstacle
        nearest_obs = calc_nearest_obs(robot_pose, obstacles)

        # Compute the scores for the generated path
        # (1) heading_angle and goal distance
        score_heading_angles = self.score_heading_angle(paths, goal_pose)
        # (2) velocity
        score_vel = self.score_vel(velocities, paths, goal_pose)
        # (3) obstacles
        score_obstacles = self.score_obstacles(paths, nearest_obs)

        # Scores Normalization
        score_heading_angles = normalize(score_heading_angles)
        score_vel = normalize(score_vel)
        score_obstacles = normalize(score_obstacles)

        # Compute the idx of the optimal path according to the overall score
        opt_idx = np.argmax(np.sum(
            np.array([score_heading_angles, score_vel, score_obstacles])
            * np.array([[self.weight_angle, self.weight_vel, self.weight_obs]]).T,
            axis=0,
        ))

        try:
            return opt_idx
        except:
            raise Exception("Not possible to find an optimal path")

    def score_heading_angle(self, path, goal_pose):
        """
        Go towards the target objective: score trajectory according to the heading angle to the goal
        """
        last_x = path[:, -1, 0]
        last_y = path[:, -1, 1]
        last_th = path[:, -1, 2]

        # calculate angle
        angle_to_goal = np.arctan2(goal_pose[1] - last_y, goal_pose[0] - last_x)

        # calculate score
        score_angle = angle_to_goal - last_th
        score_angle = np.fabs(normalize_angle(score_angle))
        score_angle = np.pi - score_angle

        return score_angle

    def score_vel(self, u, path, goal_pose):
        """
        Maximum velocity objective: score trajectory according to the forward velocity. When the robot is near the goal, slow down.
        """

        vel = u[:,0] # linear velocity value is the score itself (to maximize)
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)
        score = vel + np.exp(-dist_to_goal / self.goal_dist_tol)
        return score

    def score_obstacles(self, path, obstacles):
        """
        Obstacle avoidance objective: score trajectory according to the distance to the nearest obstacle.
        """
        score_obstacle = 2.0*np.ones((path.shape[0]))

        for obs in obstacles:
            dx = path[:, :, 0] - obs[0]
            dy = path[:, :, 1] - obs[1]
            dist = np.hypot(dx, dy)

            min_dist = np.min(dist, axis=-1)
            score_obstacle[min_dist < score_obstacle] = min_dist[min_dist < score_obstacle]
        
            # collision with obstacle
            score_obstacle[score_obstacle < self.robot.radius + self.collision_tol] = -100 # heavy penalty for collision
               
        return score_obstacle






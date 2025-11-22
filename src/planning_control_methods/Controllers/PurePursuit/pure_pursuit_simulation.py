import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from utils import DifferentialDriveRobot, proportional_control
from scipy.ndimage import rotate

from utils import RobotStates
from utils import interpolate_waypoints
from pure_pursuit import PurePursuitController
from pathlib import Path
import functools
import os
from plot_utils import create_image_marker, retrieve_image_from_path, normalize_image
from matplotlib.offsetbox import AnnotationBbox

waypoints = np.array(
    [
        [0.0, 0.0],
        [2.5, 2.5],
        [0.0, 5.0],
        [5.0, 5.0],
        [2.5, 2.5],
        [5.0, 0.0],
        [0.0, 0.0],
    ]
)

this_dir = os.path.dirname(os.path.abspath(__file__))


interpolated_waypoints = interpolate_waypoints(waypoints, resolution=0.01)

fig, ax = plt.subplots(figsize=(10, 10))

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.grid(True)
ax.set_xlim(
    np.min(interpolated_waypoints[:, 0]) - 1, np.max(interpolated_waypoints[:, 0] + 1)
)
ax.set_ylim(
    np.min(interpolated_waypoints[:, 1]) - 1, np.max(interpolated_waypoints[:, 1] + 1)
)
ax.plot(interpolated_waypoints[:, 0], interpolated_waypoints[:, 1], "b")
ax.plot(waypoints[:, 0], waypoints[:, 1], "bo")

# create the objects to plot the robot and the path that will
#  be updated during the simulation
txt_title = ax.set_title("Pure Pursuit Controller")


(robot_line,) = ax.plot([], [], color="green")
(lookahead_pt,) = ax.plot([], [], ".r")
()

robot_image_path = Path(this_dir, "images", "turtlebot3.png")
robot_image = retrieve_image_from_path(robot_image_path)

turtlebot_box = create_image_marker(robot_image, desired_width_px=20, angle=0, alpha=1)
turtlebot = AnnotationBbox(turtlebot_box, (0, 0), frameon=False)

# Simulator
#
# The following function implements a simple simulator
# that you can use to test the PurePursuit.
#
# It deals with three tasks:
#
# - simulate the robot evaluating the motion model
#   with a time resolution defined by the parameter `sim_step`
# - run the pure pursuit algorithm to compute the target velocity and angular velocity
# - **plot** the simulated robot and the target trajectory
#
# Complete the functions where you find `...`


def run_simulation(
    initial_state, target_vel, controller_dt, Lt, kp=0.5, sim_step=0.1, sim_length=120
):
    turtlebot = DifferentialDriveRobot(initial_state)  # create the robot object
    turtlebot.x = initial_state  # set the initial state
    turtlebot.u = np.array([[0.0], [0.0]])  # set the initial input

    path = interpolated_waypoints  # the path
    controller = PurePursuitController(
        turtlebot, path, 0, Lt, target_vel
    )  # create the controller object

    # convert the durations to number of time steps
    steps = int(sim_length / sim_step)
    controller_step = int(controller_dt / sim_step)
    time = 0.0  # initial time

    states = RobotStates()  # create the object to store the states
    states.append(time, controller.pind, turtlebot, a=0.0)  # store the initial state

    # The main loop that runs the simulation
    for i in range(steps):
        time += sim_step
        # Control the robot using the pure pursuit controller
        if i % controller_step == 0:
            a = proportional_control(controller.target_velocity(), turtlebot.v, kp=kp)
            w = controller.angular_velocity()
        # check any of the states is NaN or Inf
        # Update the robot state
        turtlebot.update_state([a, w], sim_step)
        states.append(time, controller.pind, turtlebot, a)
    return states


np.random.seed(42)
# initial state
initial_state = np.array([[0.0], [0.0], [1.570]])

# simulation parameters
sim_length = 120  # s
sim_step = 0.1  # s

# Parameters of the controller
target_vel = 0.22  # m/s
controller_dt = 0.5  # s
Lt = 0.5  # m
kp = 0.5  # proportional gain

# run the simulation
p_pur_states = run_simulation(
    initial_state,
    target_vel=target_vel,
    controller_dt=controller_dt,
    Lt=Lt,
    kp=kp,
    sim_step=sim_step,
    sim_length=sim_length,
)


def simulation_step(n, p_pur_states):
    """
    Simulate the robot and draw the frame for the animation
    """
    # draw animation
    robot_line.set_data([p_pur_states.x[:n]], [p_pur_states.y[:n]])
    lookahead_pt.set_data(
        [interpolated_waypoints[p_pur_states.pind[n], 0]],
        [interpolated_waypoints[p_pur_states.pind[n], 1]],
    )

    turtlebot.xyann = (p_pur_states.x[n], p_pur_states.y[n])
    turtlebot.set_animated(True)
    turtlebot_box.set_data(
        normalize_image(
            rotate(robot_image, np.degrees(p_pur_states.yaw[n]), reshape=True)
        )
    )
    # turtlebot
    txt_title.set_text("Pure Pursuit Controller, vel=%.2fm/s" % p_pur_states.v[n])
    return (robot_line, ax.add_artist(turtlebot), txt_title, lookahead_pt)


print("Creating the animation...")
# create the animation
anim = animation.FuncAnimation(
    fig,
    functools.partial(simulation_step, p_pur_states=p_pur_states),
    frames=range(0, len(p_pur_states), 10),
    blit=True,
    interval=10,
)

# save the animation
print("Saving the animation...")

save = True
if save:
    anim.save(f"{this_dir}/output_images/pure_pursuit.gif", fps=10)

plt.show()

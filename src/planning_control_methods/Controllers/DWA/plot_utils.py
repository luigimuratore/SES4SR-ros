import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib import colors


def plot_obstacles(obstacles, ax):
    # Initialize a plot and insert the landmarks
    obst_legend = ax.scatter(obstacles[:, 0], obstacles[:, 1], marker="o", c="k", s=50, label="obstacles")
    return obst_legend

def plot_trajectories(paths, ax, best_path_idx=22, N=5):
    n_paths = paths.shape[0]
       
    for i in range(0, n_paths, N):
        (sim_paths_legend,) = ax.plot(paths[i, :, 0], paths[i, :, 1], linewidth=0.8, label="sim paths")

    (best_path_legend,) = ax.plot(paths[best_path_idx, :, 0], paths[best_path_idx, :, 1], "r--", linewidth=2, label="best path")
    
    return best_path_legend

def plot_velocities(velocities):
    t = np.arange(velocities.shape[0])
    fig, axs = plt.subplots(2, 1, layout='constrained')
    axs[0].plot(t, velocities[:, 0]) # plot the linear velocities
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('v [m/s]')
    axs[0].set_title("Linear velocity")
    axs[0].grid(True)
    axs[0].legend(['v'])

    axs[1].plot(t, velocities[:, 1]) # plot the angular velocities
    axs[1].set_ylabel('w [rad/s]')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_title("Angular velocity")
    axs[1].grid(True)
    axs[1].legend(['w'])
    plt.show()
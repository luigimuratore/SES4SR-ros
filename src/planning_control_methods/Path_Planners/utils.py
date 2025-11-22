import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
from PIL import Image
import skimage
from typing import TypeVar
from enum import Enum
from typing import Tuple, Dict, Union
from pathlib import Path
from yaml import safe_load
from cv2 import imread, IMREAD_UNCHANGED
from tf_transformations import quaternion_from_euler


def get_map(map_path, scaling_factor, ax=None, plot_map=False):
    """ "
    Load the image of the 2D map and convert into numpy ndarray with xy resolution
    Return:
      - full map: np array
      - gridmap resized: np array
      - metadata: dict with the map information
    """
    # if the extension is .yaml, it is a map file
    if isinstance(map_path, str):
        print("Map path is a string")
        map_path = Path(map_path)

    if map_path.suffix == ".yaml":
        npmap, metadata = get_map_data(map_path)
    elif map_path.suffix == ".png":
        img = Image.open(map_path)
        npmap = npmap = np.asarray(img, dtype=int)
        metadata = {}

    if plot_map:
        if ax is None:
            raise ValueError("The ax parameter is required to plot the map")
        plot_gridmap(npmap, "Full Map", ax)

    # reduce the resolution: from the original map to the grid map using max pooling
    grid_map = skimage.measure.block_reduce(
        npmap, (scaling_factor, scaling_factor), np.max
    )
    grid_map_metadata = metadata.copy()
    if map_path.suffix == ".yaml":
        grid_map_metadata["resolution"] = scaling_factor * metadata["resolution"]
    grid_map_metadata["width"] = grid_map.shape[1]
    grid_map_metadata["height"] = grid_map.shape[0]
    grid_map_metadata["occupancy_gridmap"] = grid_map

    return npmap, grid_map, metadata


def orientation_around_z_axis(angle: float) -> np.ndarray:
    """
    Create a quaternion orientation around the Z axis

    :param angle: Angle to rotate
    :return: Quaternion orientation
    """
    q = quaternion_from_euler(0, 0, angle)
    return np.array(q)


class GridmapMapCost(Enum):
    """Enum class for the cost of the gridmap"""

    FREE = 0
    OCCUPIED = 100
    UNKNOWN = -1


def get_map_data(map_path: Union[Path, str]) -> Tuple[np.ndarray, Dict[str, any]]:
    """
    Get the map data from a map file

    :param map_path: Path to the map file
    :return: Occupancy gridmap and metadata
    """
    # Load data

    with open(map_path, "r") as file:
        data = safe_load(file)
    metadata = data
    # Get map data
    if type(map_path) is str:
        map_path = Path(map_path)
    map_dir = map_path.parent
    metadata["image_path"] = map_dir.joinpath(metadata.pop("image"))
    assert metadata["image_path"].exists()
    im = imread(str(metadata["image_path"]), IMREAD_UNCHANGED)
    # reflect the image to have the same orientation as the map
    im = np.flipud(im)
    metadata["occupancy_gridmap"] = image_to_occupancy_gridmap(im, metadata)
    metadata["width"] = metadata["occupancy_gridmap"].shape[1]
    metadata["height"] = metadata["occupancy_gridmap"].shape[0]

    return metadata["occupancy_gridmap"], metadata


def image_to_occupancy_gridmap(im: np.ndarray, metadata: Dict[str, any]) -> np.ndarray:
    """
    Convert an image to an occupancy gridmap

    :param im: Image to convert
    :param metadata: Metadata of the image
    :return: Occupancy gridmap
    """
    # Get map data
    if "mode" not in metadata:
        metadata["mode"] = "trinary"
    mode = metadata["mode"]
    negate = metadata["negate"]
    occupied_thresh = metadata["occupied_thresh"]
    free_thresh = metadata["free_thresh"]

    # Check if image is grayscale
    assert len(im.shape) == 2
    # check if image is normalized
    if im.max() > 1:
        im = im / 255.0

    # Convert image to occupancy gridmap
    map_data = np.full(im.shape, GridmapMapCost.UNKNOWN.value)

    image = im if negate else 1 - im
    match mode:
        case "trinary":
            map_data = np.where(
                image < free_thresh, GridmapMapCost.FREE.value, map_data
            )
            map_data = np.where(
                image > occupied_thresh, GridmapMapCost.OCCUPIED.value, map_data
            )
        case "scale":
            map_data = np.where(
                image < free_thresh, GridmapMapCost.FREE.value, map_data
            )
            map_data = np.where(
                image > occupied_thresh, GridmapMapCost.OCCUPIED.value, map_data
            )
            map_data = np.where(
                (image >= free_thresh) & (image <= occupied_thresh),
                (image - free_thresh) / (occupied_thresh - free_thresh),
                map_data,
            )
        case _:
            raise ValueError(f"Invalid mode: {mode}")
    return map_data


def world_to_map(world_pos, resolution, origin):
    """Convert world position to gridmap position
    Args:
        world_pos (np.ndarray): World position
        resolution (float): Resolution of the rescaled gridmap
        origin (np.ndarray): Origin of the gridmap
    Returns:
        np.ndarray: Gridmap position
    """

    map_pos = np.zeros(2, dtype=int)
    map_pos[0] = round((world_pos[1] - origin[1]) / resolution)
    map_pos[1] = round((world_pos[0] - origin[0]) / resolution)
    return map_pos


def map_to_world(map_pos, resolution, origin):
    """Convert gridmap position to world position
    Args:
        map_pos (np.ndarray): Gridmap position
        resolution (float): Resolution of the rescaled gridmap
        origin (np.ndarray): Origin of the gridmap
    Returns:
        np.ndarray: World position
    """

    world_pos = np.zeros(2)
    world_pos[0] = map_pos[1] * resolution + origin[0]
    world_pos[1] = map_pos[0] * resolution + origin[1]
    return world_pos


def plot_gridmap(map, title, ax):
    """Plot the gridmap with the given title
    Args:
        map (np.ndarray): Gridmap
        title (str): Title of the plot
        ax (matplotlib.axes.Axes): Axes to plot the gridmap
    """
    cmap = colors.ListedColormap(["White", "Black"])
    ax.pcolor(map[::-1], cmap=cmap)
    plot_map(map, title, ax)


def plot_costmap(costmap, title, ax):
    """Plot the costmap with the given title
    Args:
        costmap (np.ndarray): Costmap
        title (str): Title of the plot
        ax (matplotlib.axes.Axes): Axes to plot the gridmap
    """
    ax.pcolor(costmap[::-1], cmap="Blues")
    norm = colors.Normalize(vmin=0, vmax=np.max(costmap))
    sm = plt.cm.ScalarMappable(cmap="Blues", norm=norm)
    sm.set_array([])

    plot_map(costmap, title, ax)


def plot_map(map, title, ax):
    """Plot the map with the given title
    Args:
        map (np.ndarray): Map
        title (str): Title of the plot
        ax (matplotlib.axes.Axes): Axes to plot the map
    """
    if map.shape[0] < 20:
        ax.set_xticks(
            ticks=np.array(range(map.shape[1])) + 0.5, labels=range(map.shape[1])
        )
        ax.set_yticks(
            ticks=np.array(range(map.shape[0])) + 0.5,
            labels=range(map.shape[0] - 1, -1, -1),
        )
    else:
        ax.set_xticks(
            ticks=np.array(range(0, map.shape[1], int(map.shape[1] / 20))) + 0.5,
            labels=range(0, map.shape[1], int(map.shape[1] / 20)),
        )
        ax.set_yticks(
            ticks=np.array(range(0, map.shape[0], int(map.shape[0] / 20))) + 0.5,
            labels=range(map.shape[0] - 1, -1, -int(map.shape[0] / 20)),
        )

    ax.set_xlim(0, map.shape[0])
    ax.set_ylim(0, map.shape[1])
    ax.set_title(title, fontsize=14)


class Movements:
    """Base class to implement the movements"""

    def __init__(self):
        self._movements = None

    @property
    def movements(self):
        """Method to get the movements"""
        return self._movements.tolist()

    def cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the cost of the movement"""
        movement = new_pos - current_pos
        if movement in self._movements:
            return np.linalg.norm(movement, 2)
        raise ValueError(f"Invalid movement for the movement class: {type(self)}")

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        raise NotImplementedError("This method should be implemented in the subclass")


class Movements8Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array(
            [
                (1.0, 0.0),
                (1.0, 1.0),
                (0.0, 1.0),
                (-1.0, 1.0),
                (-1.0, 0.0),
                (-1.0, -1.0),
                (0.0, -1.0),
                (1.0, -1.0),
            ]
        )

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, np.inf)


class Movements4Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array([(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)])

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, 1)


SelfTypeNode = TypeVar("SelfTypeNode", bound="Node")


class Node:
    """Class to represent the nodes of the graph"""

    def __init__(self, position: np.ndarray):
        self.position = position

        self.g = 0  # cost from start node to current node
        self.h = 0  # heuristic cost from current node to end node

    @property
    def f(self):
        return self.g + self.h

    # The following methods are used to compare the nodes

    def __eq__(self, other):  # equal
        return np.array_equal(self.position, other.position)

    def __lt__(self, other):  # less than
        return self.f < other.f

    def __le__(self, other):  # less or equal
        return self.f <= other.f

    def __repr__(self):  # return a string representation of the object
        return tuple(self.position)

    def __str__(self):  # return a string representation of the object
        return f"Node {self.position} with cost {self.f}"

    def __hash__(self):  # required for instances to be usable as keys in hash tables
        return hash(tuple(self.position))

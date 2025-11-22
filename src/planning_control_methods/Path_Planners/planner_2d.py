import utils
from utils import Node


class Planner2D:
    """Base class to implement a 2D planner

    It is an abstract class to implement a 2D planner
    The class has the basic methods to implement a search-based
    planning algorithm  in a 2D grid map

    Its main methods are:
    - plan: to run the search-based algorithm
    - reset: to reset the structures of the planner

    Attributes:
        grid_map: np.array, a 2D grid map
        movements_class: A class to store the movements
        frontier: A queue to store the nodes to be visited
        came_from: A dictionary to store the path
    """

    def __init__(self, grid_map, movements_class: utils.Movements):
        self.grid_map = grid_map

        # Define the movements class to use in the planner
        # 8-connectivity movements
        # 4-connectivity movements

        self.movements_class = movements_class
        self.frontier = (
            None  # PriorityQueue() # Queue() depending on the subclass implementation
        )
        self.came_from = {}

    def plan(self, start, goal):
        """Run a search-based planning algorithm on a graph

        Args:
          start: np.array, start position
          goal: np.array, goal position

        Returns:
          Path: list of tuples
        """
        self.reset()

        # Define starting node and goal node
        start_node = Node(start)
        goal_node = Node(goal)

        # check if the start and goal nodes are valid
        if not (
            self.check_valid_position(start_node.position)
            and self.check_valid_position(goal_node.position)
        ):
            raise ValueError("Invalid start or goal node")

        # initialize frontier queue and came_from dict
        self.initialize_structures(start_node, goal_node)

        while self.frontier:
            current_node = self.frontier.get()

            if current_node == goal_node:  # Early exit condition
                break

            # Explore the graph by visiting the neighbors
            for new_position in self.movements_class.movements:
                node_position = new_position + current_node.position

                # Check if the new position is valid
                # (inside the map and not an obstacle)
                if not self.check_valid_position(node_position):
                    continue

                # Create a new node with the current node as parent
                new_node = Node(node_position)

                # Insert the new node in the frontier
                self.node_insertion(new_node, current_node, goal_node)

        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = self.came_from[current]
            return path[::-1]

        raise ValueError("No path found")

    def reset(self):
        """Method to reset the structures of the planner"""
        self.frontier = None
        self.came_from = {}

    def node_insertion(self, new_node, current, goal):
        """Method to insert a new node in the frontier"""
        raise NotImplementedError("This method should be implemented in the subclass")

    def initialize_structures(self, start_node, goal_node):
        """Method to initialize the frontier and the came_from structures"""
        raise NotImplementedError("This method should be implemented in the subclass")

    def check_on_map(self, position):
        """Method to check if a position is inside the map"""
        if (
            position[0] > (self.grid_map.shape[0] - 1)
            or position[0] < 0
            or position[1] > (self.grid_map.shape[1] - 1)
            or position[1] < 0
        ):
            return False
        return True

    def check_obstacle(self, position):
        """Method to check if a position is an obstacle"""
        if self.grid_map[tuple(position.astype(int).tolist())] > 0:
            return True
        return False

    def check_valid_position(self, position):
        """Method to check if a position is valid"""
        return self.check_on_map(position) and not self.check_obstacle(position)

    def cost_g(self, current, neighbor):
        """Method to compute the cost from the start node to the current node"""
        raise NotImplementedError("This method should be implemented in the subclass")

    def cost_h(self, current, goal):
        """Method to compute the heuristic cost from the current node to the end node"""
        raise NotImplementedError("This method should be implemented in the subclass")

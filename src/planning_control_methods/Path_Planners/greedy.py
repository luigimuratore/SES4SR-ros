from planner_2d import Planner2D
from queue import PriorityQueue


class Greedy(Planner2D):
    """Class to implement the Greedy algorithm

    It is an extension of the Planner2D class with the addition
    of the frontier attribute to store the nodes to be visited
    See the Planner2D class for more information

    Attributes:
        frontier: A priority queue to store the nodes to be visited
        came_from: A dictionary to store the path
        movements_class: A class to store the movements
    """

    def __init__(self, grid_map, movements_class):
        super().__init__(grid_map, movements_class)
        self.frontier = PriorityQueue()

    def initialize_structures(self, start_node, goal_node):
        """Method to initialize the structures of the algorithm"""
        self.frontier.put(start_node, 0)
        self.came_from[start_node] = None

    def node_insertion(self, new_node, current, goal):
        """Method to insert a new node in the frontier"""
        if new_node not in self.came_from:
            new_node.h = self.cost_h(new_node, goal)
            self.frontier.put(new_node, new_node.h)
            self.came_from[new_node] = current

    def cost_h(self, current, goal):
        """Method to compute the heuristic cost from the current node to the end node"""
        return self.movements_class.heuristic_cost(current.position, goal.position)

    def reset(self):
        super().reset()
        self.frontier = PriorityQueue()

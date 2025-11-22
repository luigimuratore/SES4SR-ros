from dijkstra import Dijkstra


class AStar(Dijkstra):
    """Class to implement the A* algorithm

    It is an extension of the Dijkstra class with the addition
    of the cost_h method to calculate the heuristic cost
    See the Dijkstra class for more information

     Attributes:
        frontier: A priority queue to store the nodes to be visited
        came_from: A dictionary to store the path
        costmap: A matrix express the cost of the map to be used in the algorithm
        cost_so_far: A dictionary to store the cost of the path
        movements_class: A class to store the movements
    """

    def node_insertion(self, new_node, current, goal):
        """Method to insert a new node in the frontier"""
        new_node.g = self.cost_so_far[current] + self.cost_g(current, new_node)
        if new_node not in self.cost_so_far or new_node.g < self.cost_so_far[new_node]:
            self.cost_so_far[new_node] = new_node.g
            new_node.h = self.cost_h(new_node, goal)
            self.frontier.put(new_node, new_node.f)
            self.came_from[new_node] = current

    def cost_h(self, current, goal):
        """Method to calculate the heuristic cost"""
        return self.movements_class.heuristic_cost(current.position, goal.position)

    def reset(self):
        """Method to reset the attributes of the class"""
        return super().reset()

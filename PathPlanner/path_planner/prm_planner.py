import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from path_planner.utils import ObstaclesGrid

class Node:
    def __init__(self, x, y):
        """
        Represents a node in the PRM roadmap.

        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
        """
        self.x = x
        self.y = y

class PRMPlanner:
    def __init__(self, start, goal, map_size, obstacles, num_samples=200, k_neighbors=10, step_size=5):
        """
        Initializes the PRM planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            num_samples (int): Number of random samples for roadmap construction.
            k_neighbors (int): Number of nearest neighbors to connect in the roadmap.
            step_size (float): Step size used for collision checking.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.step_size = step_size
        self.roadmap = [] 
        self.edges = {} 

    def construct_roadmap(self):
        """
        Constructs the probabilistic roadmap by sampling nodes and connecting them.

        Returns:
        None
        """

        pass

    def sample_free_point(self):
        """
        Samples a random collision-free point in the environment.

        Returns:
        Node: A randomly sampled node.
        """

        pass

    def find_k_nearest(self, node, k):
        """
        Finds the k-nearest neighbors of a node in the roadmap.

        Args:
            node (Node): The node for which neighbors are searched.
            k (int): The number of nearest neighbors to find.

        Returns:
            list: A list of k-nearest neighbor nodes.
        """

        pass

    def is_colliding(self, node1, node2):
        """
        Checks if the path between two nodes collides with an obstacle.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            bool: True if there is a collision, False otherwise.
        """

        pass

    def plan(self):
        """
        Plans a path from start to goal using the constructed roadmap.

        Returns:
        list: A list of (x, y) tuples representing the path.
        """

        pass

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from path_planner.utils import ObstaclesGrid
from queue import PriorityQueue


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
        self.roadmap = [self.start, self.goal]

        while len(self.roadmap) < self.num_samples + 2:
            node = self.sample_free_point()
            self.roadmap.append(node)

        for node in self.roadmap:
            self.edges[node] = []

        for node in self.roadmap:
            neighbors = self.find_k_nearest(node, self.k_neighbors)
            for neighbor in neighbors:
                if neighbor == node:
                    continue
                if not self.is_colliding(node, neighbor):
                    if neighbor not in self.edges[node]:
                        self.edges[node].append(neighbor)
                    if node not in self.edges[neighbor]:
                        self.edges[neighbor].append(node)

    def sample_free_point(self):
        """
        Samples a random collision-free point in the environment.

        Returns:
        Node: A randomly sampled node.
        """
        while True:
            x = np.random.uniform(0, self.map_size[0] - 1)
            y = np.random.uniform(0, self.map_size[1] - 1)
            if self.obstacles.is_point_valid((int(x), int(y))):
                return Node(x, y)

    def find_k_nearest(self, node, k):
        """
        Finds the k-nearest neighbors of a node in the roadmap.

        Args:
            node (Node): The node for which neighbors are searched.
            k (int): The number of nearest neighbors to find.

        Returns:
            list: A list of k-nearest neighbor nodes.
        """
        points = np.array([(n.x, n.y) for n in self.roadmap])
        tree = KDTree(points)

        query_k = min(k + 1, len(self.roadmap))
        distances, indices = tree.query((node.x, node.y), k=query_k)

        if np.isscalar(indices):
            indices = [indices]

        neighbors = []
        for idx in indices:
            candidate = self.roadmap[idx]
            if candidate is not node:
                neighbors.append(candidate)

        return neighbors[:k]

    def is_colliding(self, node1, node2):
        """
        Checks if the path between two nodes collides with an obstacle.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            bool: True if there is a collision, False otherwise.
        """
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        dist = np.hypot(dx, dy)

        if dist == 0:
            return False

        steps = max(int(dist / self.step_size), 1)

        for i in range(steps + 1):
            t = i / steps
            x = node1.x + t * dx
            y = node1.y + t * dy

            if not self.obstacles.is_point_valid((int(x), int(y))):
                return True

        return False

    def plan(self):
        """
        Plans a path from start to goal using the constructed roadmap.

        Returns:
        list: A list of (x, y) tuples representing the path.
        """
        self.construct_roadmap()

        open_set = PriorityQueue()
        open_set.put((0, self.start))

        came_from = {}
        g_cost = {node: np.inf for node in self.roadmap}
        f_cost = {node: np.inf for node in self.roadmap}

        g_cost[self.start] = 0
        f_cost[self.start] = np.hypot(self.goal.x - self.start.x, self.goal.y - self.start.y)

        visited = set()

        while not open_set.empty():
            _, current = open_set.get()

            if current in visited:
                continue

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append((current.x, current.y))
                    current = came_from[current]
                path.append((self.start.x, self.start.y))
                path.reverse()
                return path

            visited.add(current)

            for neighbor in self.edges[current]:
                if neighbor in visited:
                    continue

                edge_cost = np.hypot(neighbor.x - current.x, neighbor.y - current.y)
                tentative_g = g_cost[current] + edge_cost

                if tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g
                    h = np.hypot(self.goal.x - neighbor.x, self.goal.y - neighbor.y)
                    f_cost[neighbor] = tentative_g + h
                    open_set.put((f_cost[neighbor], neighbor))

        return []

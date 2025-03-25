import numpy as np
from scipy.spatial import KDTree

class NodeRRT:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT:
    def __init__(self):
        # Initialize RRT parameters
        self.max_iter = 2500
        self.goal_threshold = 0.15
        self.steer_range = 0.5
        self.sample_range_x = 5.0
        self.sample_range_y = 5.0

    def sample(self):
        # Sample a random point in free space
        pass

    def nearest(self, tree, sampled_point):
        # Find the nearest node on the tree to the sampled point
        pass

    def steer(self, nearest_node, sampled_point, nearest_node_idx):
        # Calculate the new node by steering from nearest_node towards sampled_point
        pass

    def plan_rrt(self, start_x, start_y, goal_x, goal_y):
        # Main RRT planning loop
        pass

    def find_path(self, tree, latest_added_node):
        # Find the path from start to goal
        pass

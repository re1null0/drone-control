#!/usr/bin/env python3

import numpy as np
from scipy.spatial import KDTree
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import rospy

class NodeRRT:
    """
    Class representing a single node in the RRT tree.
    """
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT:
    def __init__(self):
        # RRT Parameters
        self.max_iter = 2500
        self.goal_threshold = 0.15
        self.steer_range = 0.5
        self.sample_range_x = 5.0
        self.sample_range_y = 5.0
        self.current_pose = None
        self.map_updated_ = None
        
    def sample(self):
        """
        Sample a random point in free space within corridor boundaries 
        and a certain distance ahead of the car.
        """
        # Maximum attempts to find a valid point
        max_attempts = 100
        attempts = 0
        
        # Current car position and orientation
        if not self.current_pose:
            return None, None
            
        car_x, car_y, car_theta = self.current_pose
        
        while attempts < max_attempts:
            # Sample a point within the forward distance (biased toward forward direction)
            forward_distance = np.random.uniform(0.7, self.sample_range_x)
            lateral_distance = np.random.uniform(-self.sample_range_y, self.sample_range_y)
            
            # Transform to global frame
            x = car_x + forward_distance * np.cos(car_theta) - lateral_distance * np.sin(car_theta)
            y = car_y + forward_distance * np.sin(car_theta) + lateral_distance * np.cos(car_theta)
            
            # Check if the point is in free space using the occupancy grid
            if self.is_point_free(x, y):
                return (x, y)
                
            attempts += 1
            
        # If we couldn't find a valid point, try one more time with a smaller range
        x = car_x + np.random.uniform(0.3, 1.0) * np.cos(car_theta)
        y = car_y + np.random.uniform(0.3, 1.0) * np.sin(car_theta)
        
        return (x, y)
        
    def is_point_free(self, x, y):
        """
        Check if a point is in free space in the occupancy grid.
        """
        if not self.map_updated_:
            return False
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_updated_.info.origin.position.x) / self.map_updated_.info.resolution)
        grid_y = int((y - self.map_updated_.info.origin.position.y) / self.map_updated_.info.resolution)
        
        # Check if within grid bounds
        if 0 <= grid_x < self.map_updated_.info.width and 0 <= grid_y < self.map_updated_.info.height:
            # Check if cell is free (0 = free, 100 = occupied, -1 = unknown)
            idx = grid_y * self.map_updated_.info.width + grid_x
            return self.map_updated_.data[idx] == 0
            
        return False
        
    def nearest(self, tree, sampled_point):
        """
        Find the nearest node on the tree to the sampled point
        """
        nearest_node = 0
        min_dist = float('inf')
        
        for i, node in enumerate(tree):
            dist = (node.x - sampled_point[0])**2 + (node.y - sampled_point[1])**2
            if dist < min_dist:
                min_dist = dist
                nearest_node = i
                
        return nearest_node
        
    def steer(self, nearest_node, sampled_point, nearest_node_idx):
        """
        Steer from nearest node toward sampled point
        """
        # Calculate the distance between nearest node and sampled point
        dx = sampled_point[0] - nearest_node.x
        dy = sampled_point[1] - nearest_node.y
        dist = np.sqrt(dx**2 + dy**2)
        
        # If distance is less than steer range, we can reach the sampled point directly
        if dist <= self.steer_range:
            new_x = sampled_point[0]
            new_y = sampled_point[1]
        else:
            # Move in the direction of sampled point by steer_range distance
            new_x = nearest_node.x + (dx / dist) * self.steer_range
            new_y = nearest_node.y + (dy / dist) * self.steer_range
            
        new_node = NodeRRT(new_x, new_y, nearest_node_idx)
        return new_node
        
    def check_collision(self, nearest_node, new_node):
        """
        Check if path between nearest_node and new_node is collision-free
        """
        # Calculate the number of points to check along the path
        x_cell_diff = abs(int(np.ceil((nearest_node.x - new_node.x) / self.map_updated_.info.resolution)))
        y_cell_diff = abs(int(np.ceil((nearest_node.y - new_node.y) / self.map_updated_.info.resolution)))
        
        # Determine step size for interpolation
        num_steps = max(x_cell_diff, y_cell_diff)
        if num_steps == 0:  # If nodes are very close
            return not self.is_point_free(new_node.x, new_node.y)
            
        dt = 1.0 / num_steps
        t = 0.0
        
        # Check points along the path
        for i in range(num_steps + 1):
            # Interpolate between nearest_node and new_node
            x = nearest_node.x + t * (new_node.x - nearest_node.x)
            y = nearest_node.y + t * (new_node.y - nearest_node.y)
            
            # Check if this point is in collision
            if not self.is_point_free(x, y):
                return True  # Collision detected
                
            t += dt
            
        return False  # No collision
        
    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        Check if the latest added node is close enough to the goal
        """
        dist_squared = (latest_added_node.x - goal_x)**2 + (latest_added_node.y - goal_y)**2
        return dist_squared < (self.goal_threshold**2)
        
    def find_path(self, tree, latest_added_node):
        """
        Find path from start to goal
        """
        # Initialize path with the goal node
        path = [latest_added_node]
        current_node = latest_added_node
        
        # Traverse the tree backwards from the goal node to the root
        while current_node.parent is not None:
            current_node = tree[current_node.parent]
            path.append(current_node)
            
        # Reverse the path to get it from start to goal
        path.reverse()
        return path
        
    def plan_rrt(self, start_x, start_y, goal_x, goal_y):
        """
        RRT path planning algorithm implementation
        """
        # Initialize the tree with the start node
        start_node = NodeRRT(start_x, start_y)
        tree = [start_node]
        
        # Main RRT loop
        for i in range(self.max_iter):
            # Sample a random point in free space
            sampled_point = self.sample()
            if sampled_point[0] is None:
                continue
                
            # Find the nearest node in the tree
            nearest_node_idx = self.nearest(tree, sampled_point)
            
            # Steer from the nearest node toward the sampled point
            new_node = self.steer(tree[nearest_node_idx], sampled_point, nearest_node_idx)
            
            # Check if the path to the new node is collision-free
            if not self.check_collision(tree[nearest_node_idx], new_node):
                # Add the new node to the tree
                tree.append(new_node)
                
                # Check if we've reached the goal
                if self.is_goal(new_node, goal_x, goal_y):
                    return self.find_path(tree, new_node), tree
                    
        # If no path is found after max iterations
        return None, tree

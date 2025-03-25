#!/usr/bin/env python3

import numpy as np
import bezier

def bezier_curve(points, num_points=100):
    """
    Smooth the path using Bézier curves.
    
    Parameters:
    - points: List of points (x, y) to smooth.
    - num_points: Number of points to generate along the smoothed curve.
    
    Returns:
    - A list of (x, y) points representing the smoothed path.
    """
    # Convert points to numpy array
    points_array = np.array(points).T
    
    # Create a Bézier curve
    curve = bezier.Curve(points_array, degree=len(points) - 1)
    
    # Evaluate the curve at num_points evenly spaced points
    t_eval = np.linspace(0, 1, num_points)
    smoothed_points = curve.evaluate_multi(t_eval).T
    
    return smoothed_points

def smooth_trajectory(path):
    """
    Apply smoothing to the RRT path using Bézier curves.
    
    Parameters:
    - path: List of (x, y) points from the RRT path.
    
    Returns:
    - A list of (x, y) points representing the smoothed path.
    """
    # Extract x, y coordinates from path nodes
    path_points = [(node.x, node.y) for node in path]
    return bezier_curve(path_points)

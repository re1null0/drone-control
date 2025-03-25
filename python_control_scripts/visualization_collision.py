#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisualizationCollision:
    def __init__(self):
        # Initialize visualization publishers
        self.tree_viz_pub = rospy.Publisher('/rrt_tree', MarkerArray, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/rrt_path', Marker, queue_size=10)
        self.map_update_pub = rospy.Publisher('/map_updated', OccupancyGrid, queue_size=10)
        
        # Initialize CV bridge for image processing
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.process_camera_data)
        
        # Initialize map
        self.map_updated_ = None
        
    def visualize_rrt(self, path=None, tree=None):
        """
        Visualize the RRT tree and path
        """
        if not tree:
            return
            
        marker_array = MarkerArray()
        
        # Tree nodes marker
        nodes_marker = Marker()
        nodes_marker.header.frame_id = "map"
        nodes_marker.header.stamp = rospy.Time.now()
        nodes_marker.ns = "tree_nodes"
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.05
        nodes_marker.scale.y = 0.05
        nodes_marker.color.r = 0.0
        nodes_marker.color.g = 0.7
        nodes_marker.color.b = 0.0
        nodes_marker.color.a = 1.0
        
        # Tree branches marker
        branches_marker = Marker()
        branches_marker.header.frame_id = "map"
        branches_marker.header.stamp = rospy.Time.now()
        branches_marker.ns = "tree_branches"
        branches_marker.id = 1
        branches_marker.type = Marker.LINE_LIST
        branches_marker.action = Marker.ADD
        branches_marker.scale.x = 0.02
        branches_marker.color.r = 0.0
        branches_marker.color.g = 0.0
        branches_marker.color.b = 0.7
        branches_marker.color.a = 0.5
        
        # Add nodes and branches to markers
        for i, node in enumerate(tree):
            # Add node
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.1
            nodes_marker.points.append(p)
            
            # Add branch (line from parent to node)
            if i > 0 and node.parent is not None:
                # Start point (parent)
                p1 = Point()
                p1.x = tree[node.parent].x
                p1.y = tree[node.parent].y
                p1.z = 0.1
                branches_marker.points.append(p1)
                
                # End point (current node)
                p2 = Point()
                p2.x = node.x
                p2.y = node.y
                p2.z = 0.1
                branches_marker.points.append(p2)
                
        marker_array.markers.append(nodes_marker)
        marker_array.markers.append(branches_marker)
        
        self.tree_viz_pub.publish(marker_array)
        
        # Visualize path if available
        if path:
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = rospy.Time.now()
            path_marker.ns = "path"
            path_marker.id = 2
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.12
            path_marker.color.r = 1.0
            path_marker.color.g = 0.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0
            
            for node in path:
                p = Point()
                p.x = node.x
                p.y = node.y
                p.z = 0.15
                path_marker.points.append(p)
                
            self.path_viz_pub.publish(path_marker)
            
    def process_camera_data(self, image_msg):
        """
        Process camera data to detect obstacles and update the occupancy grid
        """
        if self.map_updated_ is None:
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply threshold to create binary image
            _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            
            # Find contours (potential obstacles)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create a copy of the map data for updating
            map_data = np.array(self.map_updated_.data).reshape(
                (self.map_updated_.info.height, self.map_updated_.info.width))
                
            # Process each contour
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Ignore small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Project the contour to map coordinates (this is a simplified version)
                    # In a real implementation, you would use camera calibration and perspective transform
                    grid_x = int((x - self.map_updated_.info.origin.position.x) / self.map_updated_.info.resolution)
                    grid_y = int((y - self.map_updated_.info.origin.position.y) / self.map_updated_.info.resolution)
                    
                    # Mark cells as occupied
                    if 0 <= grid_x < self.map_updated_.info.width and 0 <= grid_y < self.map_updated_.info.height:
                        map_data[grid_y, grid_x] = 100
                        
            # Update the map data
            self.map_updated_.data = map_data.flatten().tolist()
            
            # Publish updated map
            self.map_update_pub.publish(self.map_updated_)
            
        except Exception as e:
            rospy.logerr(f"Error processing camera data: {e}")

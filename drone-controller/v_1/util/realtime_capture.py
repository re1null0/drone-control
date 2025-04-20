#!/usr/bin/env python
import rospy
import numpy as np
import threading
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class OdomReceiver:
    """
    Simple subscriber that receives Odometry and stores
    the current position, velocity, and orientation in
    a shared structure for the controller to read.
    """
    def __init__(self):
        self._lock = threading.Lock()
        # Our best guess of the drone’s 'state' (x,v,q,w)
        self._state = {
            'x': np.zeros(3),
            'v': np.zeros(3),
            'q': np.array([0., 0., 0., 1.]),  # [i, j, k, w]
            'w': np.zeros(3)
        }
        self._last_stamp = rospy.Time.now()
        
        # Subscribe to whichever topic your Vicon-based odometry is published on:
        rospy.Subscriber("/vicon/drone_odom", 
                         Odometry, 
                         self._odom_callback, 
                         queue_size=1)

    def _odom_callback(self, msg):
        """
        Extracts position, velocity, orientation, etc. from
        the /vicon/drone_odom message and saves it in self._state.
        """
        with self._lock:
            # Position
            self._state['x'][0] = msg.pose.pose.position.x
            self._state['x'][1] = msg.pose.pose.position.y
            self._state['x'][2] = msg.pose.pose.position.z
            
            # Orientation (as quaternion in [i, j, k, w])
            # nav_msgs/Odometry uses geometry_msgs/Quaternion as [x, y, z, w]
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._state['q'] = np.array([qx, qy, qz, qw])
            
            # Linear velocity (assuming the odometry has a properly filled twist)
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            self._state['v'] = np.array([vx, vy, vz])
            
            # Angular velocity in rad/s
            wx = msg.twist.twist.angular.x
            wy = msg.twist.twist.angular.y
            wz = msg.twist.twist.angular.z
            self._state['w'] = np.array([wx, wy, wz])
            
            self._last_stamp = msg.header.stamp

    def get_state(self):
        """
        Returns a copy of the most recent state.
        """
        with self._lock:
            return dict(
                x = self._state['x'].copy(),
                v = self._state['v'].copy(),
                q = self._state['q'].copy(),
                w = self._state['w'].copy()
            )

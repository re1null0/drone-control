import numpy as np
from scipy.interpolate import CubicSpline

class WaypointTraj(object):
    """

    """
    def __init__(self, points, speed=.9):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        points = np.array(points)
        # Keep points properly shaped
        if points.ndim == 1:
            if points.size % 3 != 0:
                raise ValueError("points.size % 3 != 0")
            points = points.reshape(-1, 3)
        elif points.ndim == 2 and points.shape[1] != 3:
            if points.shape[0] == 3:
                points = points.T
            else:
                raise ValueError("points.shape[0] != 3")

        self.points = points
        self.speed = speed
        self.N = len(points)
        
        self.l_hat = np.diff(self.points, axis=0)
        self.segment_lengths = np.linalg.norm(self.l_hat, axis=1, keepdims=True)
        self.l_hat = self.l_hat / self.segment_lengths
        
        self.segment_times = self.segment_lengths.flatten() / self.speed
        self.t_start = np.hstack(([0], np.cumsum(self.segment_times)))
        

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """

        # STUDENT CODE HERE
        if t >= self.t_start[-1]:
            return {
                'x': self.points[-1],
                'x_dot': np.zeros(3),
                'x_ddot': np.zeros(3),
                'x_dddot': np.zeros(3),
                'x_ddddot': np.zeros(3),
                'yaw': 0,
                'yaw_dot': 0
            }
            
        segment_idx = np.searchsorted(self.t_start, t, side='right') - 1
        delta_t = t - self.t_start[segment_idx]
        
        x = self.points[segment_idx] + self.l_hat[segment_idx] * self.speed * delta_t
        x_dot = self. speed * self.l_hat[segment_idx]
        x_ddot = np.zeros(3)
        x_dddot = np.zeros(3)
        x_ddddot = np.zeros(3)
        
        yaw = 0
        yaw_dot = 0
                
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
    
    
    
''' OLD IMPLEMENTATION

x        = np.array([self.x_spline(t), self.y_spline(t), self.z_spline(t)])
x_dot    = np.array([self.x_spline.derivative()(t), self.y_spline.derivative()(t), self.z_spline.derivative()(t)])
x_ddot   = np.array([self.x_spline.derivative(2)(t), self.y_spline.derivative(2)(t), self.z_spline.derivative(2)(t)])
x_dddot  = np.array([self.x_spline.derivative(3)(t), self.y_spline.derivative(3)(t), self.z_spline.derivative(3)(t)])
x_ddddot = np.array([self.x_spline.derivative(4)(t), self.y_spline.derivative(4)(t), self.z_spline.derivative(4)(t)])

# We could set yaw and yaw_dot (omega_des) to zero but more info is better in this case
yaw = np.arctan2(x_dot[1], x_dot[0])

v_norm_sq = x_dot[0]**2 + x_dot[1]**2
if v_norm_sq > 1e-6:
    yaw_dot = (x_dot[0] * x_ddot[1] - x_dot[1] * x_ddot[0]) / v_norm_sq
else:
    yaw_dot = 0 # avouid divide by 0

flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                'yaw':yaw, 'yaw_dot':yaw_dot}
return flat_output
'''
import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by drone_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2
        lambda_ = self.k_drag / self.k_thrust

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        
        # Old
        # self.K_p = np.array([5.5, 5.5, 5.5])
        # self.K_d = np.array([6, 6, 6.75])
        self.K_p = np.array([3, 3, 6.5])
        self.K_d = np.array([2, 2, 5]) 
        self.K_i = np.array([0.2, 0.2, 0.05]) * 0

        self.K_R = np.diag([1000, 1000, 70]) # weight around 1.68
        self.K_omega = np.diag([130, 130, 11])
        self.K_R_i = np.array([0.0, 0.0, 0.0])  

        # Integral accumulator for position error
        self.int_e = np.zeros(3)

        # We’ll track last time to compute dt
        self.t_last = None

        # PD gains
        # self.K_p = np.array([35, 35, 110])
        # self.K_d = np.array([6, 6, 12])
        # self.K_R = np.diag([1000, 1000, 55])
        # self.K_omega = np.diag([160, 160, 7])
        self.M = np.linalg.inv(np.array([
            [1,                1,               1,               1               ],    
            [0,                self.arm_length, 0,               -self.arm_length],
            [-self.arm_length, 0,               self.arm_length, 0               ],
            [lambda_,          -lambda_,        lambda_,         -lambda_        ]
        ]))
        
        print(f"SE3Control initialized with quad_params: {quad_params}")

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        # cmd_motor_speeds = np.zeros((4,))
        # cmd_thrust = 0
        # cmd_moment = np.zeros((3,))
        # cmd_q = np.zeros((4,))
        # 1) Compute dt for integral
        if self.t_last is None:
            dt = 0.0
        else:
            dt = t - self.t_last
        self.t_last = t

        # 2) Position and velocity errors
        pos_error = state['x'] - flat_output['x']
        vel_error = state['v'] - flat_output['x_dot']

        # 3) Integral of position error
        self.int_e += pos_error * dt # * 0 # simple Euler integration

        # print(f"error: {self.int_e}")
        # Optional: clamp/saturate self.int_e to avoid integral windup

        # 1. Calculate F_des
        r_dd_des = (flat_output['x_ddot']
                - self.K_p * pos_error
                - self.K_d * vel_error
                - self.K_i * self.int_e)
        F_des = self.mass * r_dd_des + np.array([0, 0, self.mass * self.g])
        
        # 2. Compute u_1 or cmd_thrust
        quaternion = state['q']
        R = Rotation.from_quat(quaternion).as_matrix()
        b_3 = R[:,2]
        u_1 = np.dot(b_3, F_des)
        
        # 3. Compute R_des and b_i_des
        b_3_des = F_des / np.linalg.norm(F_des)
        a_psi = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0])
        b_2_des = np.cross(b_3_des, a_psi) / np.linalg.norm(np.cross(b_3_des, a_psi))
        b_1_des = np.cross(b_2_des, b_3_des)
        R_des = np.vstack((b_1_des, b_2_des, b_3_des)).T
        
        # 4. Find e_R and substitute omega for e_omega
        error_matrix = R_des.T @ R - R.T @ R_des
        e_R = 0.5 * np.array([error_matrix[2,1], error_matrix[0,2], error_matrix[1,0]]) # apply vee operator
        # e_omega = state['w'] - flat_output['yaw_dot']
        e_omega = state['w'] - np.array([0.0, 0.0, flat_output['yaw_dot']])

        # 5. Compute u_2 or cmd_moment
        u_2 = self.inertia @ (-self.K_R @ e_R - self.K_omega @ e_omega)
        
        U = np.array([u_1, u_2[0], u_2[1], u_2[2]])
        
        omega_sq = np.clip(self.M @ U / self.k_thrust, 0, self.rotor_speed_max**2) # ensure positive
        omega = np.clip(np.sqrt(omega_sq), self.rotor_speed_min, self.rotor_speed_max)
        
        cmd_q = Rotation.from_matrix(R_des).as_quat()  

        control_input = {'cmd_motor_speeds':omega,
                         'cmd_thrust':u_1,
                         'cmd_moment':u_2,
                         'cmd_q':cmd_q}
        return control_input

import numpy as np
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import time

from pymavlink import mavutil

from flightsim.simulate import Quadrotor, simulate
from flightsim.drone_params import quad_params
from flightsim import hover_traj

import waypoint_traj
import se3_control
from pixhawk_realtime import Pixhawk_Control

# This object defines the quadrotor dynamical model and should not be changed.
quadrotor = Quadrotor(quad_params) # GIVEN

# You will complete the implementation of the SE3Control object.
my_se3_control = se3_control.SE3Control(quad_params) # GIVEN
t_final = 20 # CEDRIC GOT IT
t_array = np.linspace(0, t_final, t_final)

points = np.array([
    [0, 0, 0],
    [0, 0, 1],
    [0, 0, 1],
    [0, 0.5, 1],
    [0.5, 0.5, 1],
    [0.5, -0.5, 1],
    [-0.25, -0.5, 1.5]
    ])
my_traj = waypoint_traj.WaypointTraj(points)

''' x: x,y,z position at t=0
    v: x,y,z velocity at t=0
    q: quaternion - for orientation information at t=0 -- cedric about to give a converter
    w: x,y,z angular velocity at t=0'''
    
initial_state = {'x': np.array([0, 0, 0]),
                 'v': np.zeros(3,),
                 'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
                 'w': np.zeros(3,)}

# Perform simulation
print('Simulate.')
(time, state, control, flat, exit) = simulate(initial_state,  # Needs an input   
                                              quadrotor,      # Always the same
                                              my_se3_control, # Always the same
                                              my_traj,        # Needs an input (Cedric got it)
                                              t_final)        # Cedric got it
print(exit.value)
# print(f"Control: {control}")

# Plot Results
graphing = False

if graphing:
    import matplotlib
    #matplotlib.use('TkAgg') 
    import matplotlib.pyplot as plt

    from flightsim.animate import animate
    from flightsim.world import World
    from flightsim.axes3ds import Axes3Ds


    # Set simulation parameters.
    w = 2
    world = World.empty((-w, w, -w, w, -w, w))

    # Position and Velocity vs. Time
    (fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Position vs Time')
    x = state['x']
    x_des = flat['x']
    ax = axes[0]
    ax.plot(time, x_des[:,0], 'r', time, x_des[:,1], 'g', time, x_des[:,2], 'b')
    ax.plot(time, x[:,0], 'r.',    time, x[:,1], 'g.',    time, x[:,2], 'b.')
    ax.legend(('x', 'y', 'z'))
    ax.set_ylabel('position, m')
    ax.grid('major')
    ax.set_title('Position')
    v = state['v']
    v_des = flat['x_dot']
    ax = axes[1]
    ax.plot(time, v_des[:,0], 'r', time, v_des[:,1], 'g', time, v_des[:,2], 'b')
    ax.plot(time, v[:,0], 'r.',    time, v[:,1], 'g.',    time, v[:,2], 'b.')
    ax.legend(('x', 'y', 'z'))
    ax.set_ylabel('velocity, m/s')
    ax.set_xlabel('time, s')
    ax.grid('major')
    plt.savefig("real_time_quad_pos_vel.png")

    # Orientation and Angular Velocity vs. Time
    (fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Orientation vs Time')
    q_des = control['cmd_q']
    q = state['q']
    ax = axes[0]
    ax.plot(time, q_des[:,0], 'r', time, q_des[:,1], 'g', time, q_des[:,2], 'b', time, q_des[:,3], 'k')
    ax.plot(time, q[:,0], 'r.',    time, q[:,1], 'g.',    time, q[:,2], 'b.',    time, q[:,3],     'k.')
    ax.legend(('i', 'j', 'k', 'w'))
    ax.set_ylabel('quaternion')
    ax.set_xlabel('time, s')
    ax.grid('major')
    w = state['w']
    ax = axes[1]
    ax.plot(time, w[:,0], 'r.', time, w[:,1], 'g.', time, w[:,2], 'b.')
    ax.legend(('x', 'y', 'z'))
    ax.set_ylabel('angular velocity, rad/s')
    ax.set_xlabel('time, s')
    ax.grid('major')
    plt.savefig("real_time_quad_pos_vel.png")

    # Commands vs. Time
    (fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Commands vs Time')
    s = control['cmd_motor_speeds']
    ax = axes[0]
    ax.plot(time, s[:,0], 'r.', time, s[:,1], 'g.', time, s[:,2], 'b.', time, s[:,3], 'k.')
    ax.legend(('1', '2', '3', '4'))
    ax.set_ylabel('motor speeds, rad/s')
    ax.grid('major')
    ax.set_title('Commands')
    M = control['cmd_moment']
    ax = axes[1]
    ax.plot(time, M[:,0], 'r.', time, M[:,1], 'g.', time, M[:,2], 'b.')
    ax.legend(('x', 'y', 'z'))
    ax.set_ylabel('moment, N*m')
    ax.grid('major')
    T = control['cmd_thrust']
    ax = axes[2]
    ax.plot(time, T, 'k.')
    ax.set_ylabel('thrust, N')
    ax.set_xlabel('time, s')
    ax.grid('major')
    plt.savefig("real_time_quad_pos_vel.png")

    # 3D Paths
    fig = plt.figure('3D Path')
    ax = Axes3Ds(fig)
    world.draw(ax)
    ax.plot3D(state['x'][:,0], state['x'][:,1], state['x'][:,2], 'b.')
    ax.plot3D(flat['x'][:,0], flat['x'][:,1], flat['x'][:,2], 'k')
    plt.savefig("real_time_quad_pos_vel.png")

    # Animation (Slow)
    # Instead of viewing the animation live, you may provide a .mp4 filename to save.
    R = Rotation.from_quat(state['q']).as_matrix()

    ani = animate(time, state['x'], flat['x'], R, world=world, filename=None)
    print(f"-----------------------------")
    print(f"Displaying Simulation Results")
    print(f"-----------------------------\n")
    input("Review_animation to continue...\n")

Pixhawk = Pixhawk_Control(quad_params)
# Connect to Pixhawk
master = Pixhawk.connect_and_setup()

# Then the loop for replay_in_real_time:
Pixhawk.replay_in_real_time(master, time, state, control, flat)

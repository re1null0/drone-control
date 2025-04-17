#!/usr/bin/env python3
import rospy
import numpy as np
import time

from pixhawk_realtime import Pixhawk_Control
import waypoint_traj
from se3_control import SE3Control
from flightsim.drone_params import quad_params  # holds mass, Ixx, etc.

# If you want to track a reference path, you can import your trajectory:
# from waypoint_traj import WaypointTraj

# The OdomReceiver you wrote:
from realtime_capture import OdomReceiver

def real_time_control_loop():
    rospy.init_node('real_time_controller', anonymous=True)
    
    # A high-ish update rate, e.g. 100 Hz
    rate_hz = 100
    rate = rospy.Rate(rate_hz)
    
    # 1) Subscribe to Vicon-based odometry
    odom_sub = OdomReceiver()

    # 2) Initialize the SE3 controller & load any gains
    controller = SE3Control(quad_params)

    # 3) Connect to the Pixhawk (MAVLink) 
    pix = Pixhawk_Control(quad_params)
    master = pix.connect_and_setup()
    
    points = np.array([
        [0, 0, 0],
        [0, 0, 1]
        # [0, 0, 1],
        # [0.5, 0, 1],
        # [1, 0, 0.5],
        # [1, 0, 0]
        # [0, 0.5, 1],
        # [0.5, 0.5, 1],
        # [0.5, -0.5, 1],
        # [-0.25, -0.5, 1.5]
        ])
    my_traj = waypoint_traj.WaypointTraj(points)
    
    t0 = time.time()

    xyz_offset = [-0.00599, -0.04815, -0.2797]

    while not rospy.is_shutdown():
        now = time.time() - t0
        
        # 1) Get the current drone state from the Vicon
        current_state = odom_sub.get_state()
        current_state['x'] += xyz_offset
                
        # 2) Build a desired 'flat_output'
        flat_output = my_traj.update(now)
        
        # 3) Compute control: cmd_motor_speeds, cmd_thrust, cmd_moment, cmd_q
        control_input = controller.update(now, current_state, flat_output)

        # 4) Send that command to the Pixhawk
        cmd_thrust = control_input['cmd_thrust']  # in Newtons
        cmd_q      = control_input['cmd_q']       # [i, j, k, w]
        
        # Convert them to Euler angles + normalized thrust, and send
        pix.send_rt_command(master, cmd_q, cmd_thrust, now, current_state, flat_output)
        
        
        # 5) Sleep to maintain loop rate
        rate.sleep()
    
    # Optionally disarm upon exit
    print("Simulation Completed.")
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #     0,
    #     0, 0, 0, 0, 0, 0, 0
    # )
    # master.motors_disarmed_wait()
    # print("Motors disarmed. Shutting down.")

if __name__=="__main__":
    real_time_control_loop()

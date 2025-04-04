#!/usr/bin/env python3

import time
from scipy.spatial.transform import Rotation as R
import numpy as np
from pymavlink import mavutil
from tqdm import tqdm

##############################################
# 1) MAVLink Connection & Setup
##############################################
def connect_and_setup(connection_string='udp:127.0.0.1:14551', baud=115200):
    print(f"Connecting to {connection_string}...")
    master = mavutil.mavlink_connection(connection_string, baud=baud)
    master.wait_heartbeat()
    print("Heartbeat received "
          f"(system {master.target_system}, component {master.target_component})")

    # Disable arming checks
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)

    # Optionally set GUIDED_NOGPS "GUID_OPTIONS" so it will accept attitude control
    print("Enabling external attitude control (GUID_OPTIONS = 7)...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'GUID_OPTIONS',
        float(7),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)

    # Switch mode to GUIDED_NOGPS
    print("Switching to GUIDED_NOGPS mode...")
    mode_id = master.mode_mapping()['GUIDED_NOGPS']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1)

    # Arm the motors
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed!")
    return master

##############################################
# 2) Euler <-> Quaternion Helpers
##############################################
def euler_to_quaternion(roll_rad, pitch_rad, yaw_rad):
    """
    roll_rad, pitch_rad, yaw_rad in radians
    Return quaternion in the order [w, x, y, z]
    suitable for MAVLink's set_attitude_target (ArduPilot).
    """
    # SciPy's default .as_quat() gives [x, y, z, w].
    # So we reorder to [w, x, y, z].

    # Create a rotation from euler angles (XYZ convention)
    rot_obj = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
    xyzw = rot_obj.as_quat()  # [x, y, z, w]
    return [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]

def quaternion_to_euler(q_wxyz):
    """
    q_wxyz in [w, x, y, z] format
    Return (roll, pitch, yaw) in radians, using 'xyz' convention.
    """
    # SciPy expects [x, y, z, w].
    xyzw = [q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]
    rot_obj = R.from_quat(xyzw)
    roll, pitch, yaw = rot_obj.as_euler('xyz')
    return (roll, pitch, yaw)

##############################################
# 3) Send Attitude & Thrust
##############################################
def send_attitude_target(master, roll_rad, pitch_rad, yaw_rad, thrust):
    """
    thrust is 0..1 range for ArduPilot
    """
    # Convert Euler -> quaternion [w, x, y, z]
    q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

    # Bits:
    # 0: body roll rate
    # 1: body pitch rate
    # 2: body yaw rate
    # 6: attitude
    # 7: thrust
    # We'll ignore body rates, so set bits 0,1,2
    type_mask = (1 << 0) | (1 << 1) | (1 << 2)

    # Time in milliseconds
    now_ms = int(time.time() * 1000) & 0xFFFFFFFF

    master.mav.set_attitude_target_send(
        now_ms,                         # time_boot_ms
        master.target_system,
        master.target_component,
        type_mask,
        q,                              # [w, x, y, z]
        0.0, 0.0, 0.0,                  # roll/pitch/yaw rates (ignored)
        thrust
    )

##############################################
# 4) Main "Replay/Real-Time" Loop
##############################################
def replay_in_real_time(master,
                        time_array,
                        state_dict,
                        control_dict):
    """
    Steps through the simulation output arrays
    (time, state, and control) in real-time, sending
    attitude+thrust commands to the Pixhawk.

    Parameters:
      master      -- mavutil connection from connect_and_setup()
      time_array  -- 1D array of time instants from your sim, shape (N,)
      state_dict  -- dictionary with keys like 'x','v','q', etc. from your sim
      control_dict-- dictionary with keys like 'cmd_thrust', 'cmd_q', etc.
    """

    # We assume:
    #  - control_dict['cmd_q'][i] = [i, j, k, w] (quaternion)
    #  - control_dict['cmd_thrust'][i] in Newtons
    #  - Possibly control_dict['cmd_motor_speeds'][i] if you want to see them

    # Example: Suppose the drone’s weight is mg ~ 9.81N for a 1 kg vehicle.
    # If 'cmd_thrust' is near 9.81, that might be "hover" ~ 0.5 in ArduPilot.
    # You might do a rough scaling here:
    vehicle_mass = 1.0      # user-chosen or from config
    g = 9.81
    hover_newtons = vehicle_mass*g   # ~9.81 for 1 kg
    # scale thrust from N -> 0..1
    def scale_thrust(thrust_N):
        return thrust_N / (2.0 * hover_newtons)  # naive example scale

    # TQDM for progress bar
    print("Starting real-time attitude streaming to Pixhawk...\n")
    start_real_time = time.time()

    for i in tqdm(range(len(time_array)), desc="Sending commands"):
        # 1) figure out dt from sim
        if i == 0:
            dt = 0.0
        else:
            dt = time_array[i] - time_array[i-1]

        # 2) current target from the sim
        sim_q = control_dict['cmd_q'][i]   # [i, j, k, w]
        sim_thrust_N = control_dict['cmd_thrust'][i]

        # convert the quaternion from [i, j, k, w] to euler angles
        roll, pitch, yaw = quaternion_to_euler(sim_q)  # radians

        # scale the thrust from N to [0..1]
        thrust_01 = scale_thrust(sim_thrust_N)

        # 3) send it to Pixhawk
        send_attitude_target(master, roll, pitch, yaw, thrust_01)

        # 4) (Optional) print or log local state, but keep console from filling
        #    We'll just do it once in a while.
        if i % max(int(len(time_array) / 20),1) == 0:
            # Single-line update with \r. If you prefer, keep tqdm only.
            x = state_dict['x'][i]
            print(f"t={time_array[i]:5.2f}   pos=({x[0]:4.2f}, {x[1]:4.2f}, {x[2]:4.2f})   "
                  f"thrust={sim_thrust_N:5.2f}N => {thrust_01:4.2f}",
                  end='\r')

        # 5) Sleep until next step. 
        #    That is, we want actual wall-clock time to match sim time intervals
        time.sleep(dt)

    print("\nDone streaming commands!\n")

##############################################
# 5) Putting it all together
##############################################
def main():
    # 1) Connect and setup
    master = connect_and_setup()

    # 2) Suppose you have run your simulation to get:
    #    time, state, control
    #    from something like: (time, state, control, flat, exit) = simulate(...)
    #    We'll assume you can import or call your function to get that.
    #    For demonstration, let's do some fake arrays:
    N = 100
    time_sim = np.linspace(0, 10, N)  # 10-second sim
    # Example placeholders:
    #   state['x'] = Nx3
    #   control['cmd_q'] = Nx4
    #   control['cmd_thrust'] = Nx1
    state_sim = {
        'x': np.column_stack([np.linspace(0, 1, N),
                              np.linspace(0, 2, N),
                              np.linspace(1, 2, N)]),
        # ...
    }
    control_sim = {
        'cmd_q': np.tile([0, 0, 0, 1], (N, 1)),   # identity orientation
        'cmd_thrust': np.linspace(0, 15, N),     # ramp from 0N to 15N
        # ...
    }

    # 3) Send them out in real time
    replay_in_real_time(master, time_sim, state_sim, control_sim)

    # 4) Disarm at end
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed. All done.")

if __name__ == "__main__":
    main()
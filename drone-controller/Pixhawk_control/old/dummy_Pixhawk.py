#!/usr/bin/env python3
import time
import numpy as np
from pymavlink import mavutil

# --- User-defined parameters (should match your quad_params) ---
rotor_speed_min = 100  # [rad/s] (example value)
rotor_speed_max = 700  # [rad/s] (example value)

def normalize_motor_command(omega):
    """
    Convert a motor speed (rad/s) to a normalized value (0 to 1)
    based on the minimum and maximum rotor speeds.
    """
    norm_val = (omega - rotor_speed_min) / (rotor_speed_max - rotor_speed_min)
    return float(np.clip(norm_val, 0, 1))

def get_motor_outputs(t):
    """
    Example function returning an array of 4 motor speeds (in rad/s).
    Sinusoidal pattern for demonstration.
    """
    base = (rotor_speed_max + rotor_speed_min) / 2.0
    amp  = (rotor_speed_max - rotor_speed_min) / 2.0
    motor_speeds = base + amp * np.array([np.sin(t),
                                          np.sin(t + 0.5),
                                          np.sin(t + 1.0),
                                          np.sin(t + 1.5)])
    return motor_speeds

def arm(master):
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print("Motors armed")

def disarm(master):
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print("Motors disarmed")

def set_mode(master, mode_name):
    print(f"Setting mode to {mode_name}")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def disable_arm_checks(master):
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(2)

# --- MAVLink connection setup ---
connection_string = 'udp:127.0.0.1:14551'
baud_rate = 115200
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

print("Waiting for heartbeat from Pixhawk...")
master.wait_heartbeat()
print("Heartbeat received. Connected to Pixhawk.")

# Switch to GUIDED mode (be sure your Pixhawk can accept actuator cmds here)
set_mode(master, 'GUIDED')

# Optionally disable checks
disable_arm_checks(master)

# Arm
arm(master)

# Main control loop
t_final = 20.0
dt = 0.02  # 50 Hz
start_time = time.time()

print("Starting offboard motor control (raw actuator commands).\n")
while True:
    now = time.time()
    t_sim = now - start_time
    if t_sim > t_final:
        break

    motor_speeds = get_motor_outputs(t_sim)

    # Normalize each motor command to [0,1]
    norm_commands = [normalize_motor_command(omega) for omega in motor_speeds]

    # Create an array of 8 controls (only the first 4 used for main motors)
    controls = norm_commands + [0.0]*4
    print(f"controls: {controls}", end="\r")
    master.mav.set_actuator_control_target_send(
        int((now - start_time)*1e6),  # send time in microseconds since start
        master.target_system,
        master.target_component,
        0,  # group_mlx: 0 for main outputs
        controls
    )
    time.sleep(dt)

# Disarm at the end
disarm(master)
print("Motor command sequence complete.")

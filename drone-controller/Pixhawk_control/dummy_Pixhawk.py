#!/usr/bin/env python3
import time
import numpy as np
from pymavlink import mavutil

# --- User-defined parameters (should match your quad_params) ---
rotor_speed_min = 100  # [rad/s] (example value)
rotor_speed_max = 800  # [rad/s] (example value)

def normalize_motor_command(omega):
    """
    Convert a motor speed (rad/s) to a normalized value (0 to 1)
    based on the minimum and maximum rotor speeds.
    """
    norm_val = (omega - rotor_speed_min) / (rotor_speed_max - rotor_speed_min)
    return np.clip(norm_val, 0, 1)

# --- Example simulation function ---
def get_motor_outputs(t):
    """
    Replace this dummy function with your actual simulation output.
    Returns an array of 4 motor speeds (in rad/s).
    For demonstration, we use a simple sinusoidal variation.
    """
    # Example: motor outputs vary sinusoidally between rotor_speed_min and rotor_speed_max.
    base = (rotor_speed_max + rotor_speed_min) / 2.0
    amp  = (rotor_speed_max - rotor_speed_min) / 2.0
    motor_speeds = base + amp * np.array([np.sin(t),
                                          np.sin(t + 0.5),
                                          np.sin(t + 1.0),
                                          np.sin(t + 1.5)])
    return motor_speeds

# --- MAVLink connection setup ---
# Adjust the connection string as appropriate.
# Here we use UDP outbound from the companion (Jetson Nano) to Pixhawk.
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
print("Waiting for heartbeat from Pixhawk...")
master.wait_heartbeat()
print("Heartbeat received. Connected to Pixhawk.")

# Optionally, disable any internal control loops (if needed) and ensure offboard mode is enabled.

# --- Main control loop ---
t_final = 20.0   # total duration (seconds)
dt = 0.02        # control loop time step (50 Hz)
start_time = time.time()

print("Starting offboard motor control (blind mode)...")
while True:
    current_time = time.time()
    t_sim = current_time - start_time
    if t_sim > t_final:
        break

    # Obtain the simulation-computed motor speeds (4-element array, in rad/s)
    motor_speeds = get_motor_outputs(t_sim)
    
    # Normalize each motor command to a value in [0, 1]
    norm_commands = [float(normalize_motor_command(omega)) for omega in motor_speeds]
    
    # Create an array of 8 controls (only the first 4 are used for the main motor group)
    controls = norm_commands + [0.0] * 4  # total 8 elements
    
    # Send the actuator control target message.
    # This message sets the actuator outputs for the group (here, group 0 = main motors).
    master.mav.set_actuator_control_target_send(
        int(current_time * 1e6),  # time in microseconds
        master.target_system,
        master.target_component,
        0,         # actuator group (0 for main outputs)
        controls   # list of 8 float control values (normalized)
    )
    
    # Sleep until the next cycle
    time.sleep(dt)

print("Motor command sequence complete.")
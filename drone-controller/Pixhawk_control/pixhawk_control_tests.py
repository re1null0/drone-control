import numpy as np
import time
from v_1.code.pixhawk_realtime import Pixhawk_Control
from flightsim.drone_params import quad_params

from pymavlink import mavutil

pix = Pixhawk_Control(quad_params)
connection_string='/dev/ttyACM0'
baud=115200

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


now = 0
t0 = time.time()

while now < 3:
    print("All motors")
    now = time.time() - t0
    pix.send_attitude_target(master, [1, 0, 0, 0], 0.7)

while now < 6:
    print(f"motor index{0}")
    now = time.time() - t0
    throttle_pct = 75
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0, 
        1, # Motor idx 1-4
        0, # throttle type set to percentage
        throttle_pct,
        0, # timeout (s)
        0, 0, 0       
    )

# while now < 6:
#     print("Left motors go crazy")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [0.71, 0.71, 0, 0], 0.2)

# while now < 9:
#     print("Right motors go crazy")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [0.71, -0.71, 0, 0], 0.2)

# print("\nNow opposite configs\n")

# while now < 12:
#     print("All motors")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [0, 0, 0, 0], 0.2)

# while now < 15:
#     print("Left motors go crazy")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [0, 0.71, 0, 0.71], 0.2)

# while now < 18:
#     print("Right motors go crazy")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [0, -0.71, 0, 0.71], 0.2)
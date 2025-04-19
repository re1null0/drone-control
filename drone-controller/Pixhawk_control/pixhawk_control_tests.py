import numpy as np
import time
from v_1.code.pixhawk_realtime import Pixhawk_Control
from flightsim.drone_params import quad_params

from pymavlink import mavutil

pix = Pixhawk_Control(quad_params)
master = pix.connect_and_setup()

now = 0
t0 = time.time()

# while now < 3:
#     print("All motors")
#     now = time.time() - t0
#     pix.send_attitude_target(master, [1, 0, 0, 0], 0.7)

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
from pymavlink import mavutil

drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

drone.wait_heartbeat()
print("Heartbeat received!")

mode = 'GUIDED'
mode_id = drone.mode_mapping()[mode]
drone.set_mode(mode_id)


drone.arducopter_arm()
drone.motors_armed_wait()
print("Drone armed and ready.")


drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,              # Confirmation
    0, 0, 0,        # Pitch, Yaw, empty
    0, 0,           # Latitude, Longitude (0 = current)
    2               # Altitude in meters
)
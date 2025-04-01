from pymavlink import mavutil
import time

# Connect to Pixhawk
connection_string = '/dev/ttyACM0'
baud_rate = 115200
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for the heartbeat msg to find the system ID
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Function to arm the drone
def arm():
    print("Arming motors")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    # Wait for arming confirmation
    master.motors_armed_wait()
    print(" Motors armed")

# Function to take off
def takeoff(altitude):
    print(f" Taking off to {altitude} meters")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)

# Function to land
def land():
    print("Landing")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)

# Set mode to GUIDED
def set_guided_mode():
    print("Setting GUIDED mode")
    mode = 'GUIDED'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

# MAIN SCRIPT
set_guided_mode()
time.sleep(2)

arm()
time.sleep(2)

takeoff(2)  # Take off to 2 meters
time.sleep(10)  # Hover for 10 seconds

land()

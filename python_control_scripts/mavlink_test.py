from pymavlink import mavutil
import time

# Connect to Pixhawk
connection_string = '/dev/ttyACM0'
baud_rate = 115200
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
hover_time = 5      # seconds to hover
ascend_time = 3     # seconds to ascend
ascent_speed = -1.5  # m/s, NEGATIVE = up in NED

boot_time = time.time()


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
def set_mode(mode_name):
    print("Setting GUIDED mode")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def send_velocity(vx, vy, vz, duration):
    print(f"Sending velocity: vx={vx}, vy={vy}, vz={vz} for {duration}s")
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    for _ in range(int(duration * 10)):  # 10Hz
        master.mav.set_position_target_local_ned_send(
            int((time.time() - boot_time) * 1000),
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # or BODY_NED
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        time.sleep(0.1)

def override_throttle(throttle_pwm):
    print(f"Overriding throttle with PWM={throttle_pwm}")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, throttle_pwm, 0, 0, 0, 0, 0)  # Channel 3 is usually throttle


def disable_arm_checks():
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(2)

def listen_for_feedback(timeout=5):
    print("\nListening for responses from Pixhawk...\n")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(blocking=False)
        if msg:
            print(f"Received: {msg.get_type()} → {msg}")
        time.sleep(0.1)


# MAIN SCRIPT
set_mode('GUIDED_NOGPS')
disable_arm_checks()
arm()
time.sleep(3)
send_velocity(0, 0, -1.0, 3)  # Aggressive upward command
send_velocity(0, 0, 0, 2)     # Hover
land()
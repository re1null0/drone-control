from pymavlink import mavutil
import time

connection_string = '/dev/ttyACM0'
baud_rate = 115200
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

boot_time = time.time()
ascent_speed = -1.0  # m/s (up in NED)
ascend_time = 3      # seconds


def wait_heartbeat():
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))


def disable_arm_checks():
    print("Disabling arming checks...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(2)


def set_mode(mode_name):
    print(f"Setting mode to {mode_name}...")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    time.sleep(2)

    # Confirm mode switch
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print("Current mode:", mavutil.mode_string_v10(msg))


def arm():
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print("Motors armed!")


def set_fake_home():
    print("Setting fake home location (0,0,0)...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)


def check_ekf_status():
    print("Requesting EKF status...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        0, 0, 0, 0, 0, 0)

    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=3)
    if msg:
        print("EKF STATUS:", msg)
    else:
        print("⚠️ No EKF status received")


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
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,        # Position
            vx, vy, vz,     # Velocity
            0, 0, 0,        # Acceleration
            0, 0)           # Yaw, yaw rate
        time.sleep(0.1)


def listen_for_feedback(timeout=5):
    print("\nListening for MAVLink feedback...\n")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(blocking=False)
        if msg:
            print(f"{msg.get_type()}: {msg}")
        time.sleep(0.1)


def set_fake_ekf_origin():
    print("Setting fake EKF origin...")
    
    # Send fake GPS origin (required even in GUIDED_NOGPS)
    master.mav.set_gps_global_origin_send(
        master.target_system,
        0, 0, 0,  # lat, lon, alt = 0 (use valid values if you want)
        0         # target_component
    )

    # Set home position too
    master.mav.set_home_position_send(
        master.target_system,
        0, 0, 0,      # lat, lon, alt
        0, 0, 0,      # x, y, z
        [1, 0, 0, 0], # orientation quaternion
        0, 0, 0       # approach
    )

    time.sleep(2)



# === MAIN FLOW ===
wait_heartbeat()
disable_arm_checks()
set_mode('GUIDED_NOGPS')
set_fake_home()
arm()
set_fake_ekf_origin
check_ekf_status()

time.sleep(1)
send_velocity(0, 0, -2, 10)
listen_for_feedback(5)

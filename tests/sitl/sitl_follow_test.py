# RESTORED PROJECT TEST ARTIFACT
# This file was restored from the project's original ChatGPT code history.
# Constants, filenames, phase purpose and control structure are preserved wherever
# the history retained them. Where the original source bytes were unavailable,
# glue/boilerplate was reconstructed to make the historical test a standalone file.
# EXPERIMENTAL UAV SOFTWARE — review configuration and test safely.

from pymavlink import mavutil
import math
import time

TAKEOFF_ALTITUDE = 3.0
TARGET_DISTANCE = 3.0
DISTANCE_TOLERANCE = 0.4
SCAN_YAW_RATE_DEG = 25.0
FACE_YAW_DEG = 20.0
FORWARD_SPEED = 0.6
BACKWARD_SPEED = -0.6

master = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
master.wait_heartbeat()
print("SITL connected")

def send_body_velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate_deg=0.0, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            1479,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, math.radians(yaw_rate_deg),
        )
        time.sleep(0.1)

def get_altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def takeoff(altitude):
    master.set_mode_apm("GUIDED")
    master.arducopter_arm()
    master.motors_armed_wait()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude,
    )
    while True:
        current = get_altitude()
        if current is not None:
            print(f"Altitude: {current:.2f}m")
            if current >= altitude * 0.85:
                return
        time.sleep(0.2)

def fake_person(scan_count):
    if scan_count >= 5:
        return True, 0.30, 4.5
    return False, None, None

def land_and_disarm():
    master.set_mode_apm("LAND")
    print("LAND")
    time.sleep(10)
    master.arducopter_disarm()
    try:
        master.motors_disarmed_wait()
    except Exception:
        pass
    master.set_mode_apm("GUIDED")

try:
    takeoff(TAKEOFF_ALTITUDE)

    detected = False
    for scan_count in range(12):
        detected, x, distance = fake_person(scan_count)
        if detected:
            print(f"Fake person detected: x={x:.2f}, distance={distance:.1f}m")
            break
        print("Scanning...")
        send_body_velocity(yaw_rate_deg=SCAN_YAW_RATE_DEG, duration=1.0)

    if detected:
        if x < 0:
            send_body_velocity(yaw_rate_deg=-FACE_YAW_DEG, duration=2.0)
        elif x > 0:
            send_body_velocity(yaw_rate_deg=FACE_YAW_DEG, duration=2.0)

        if distance > TARGET_DISTANCE + DISTANCE_TOLERANCE:
            print("MOVE FORWARD")
            send_body_velocity(vx=FORWARD_SPEED, duration=2.0)
        elif distance < TARGET_DISTANCE - DISTANCE_TOLERANCE:
            print("MOVE BACK")
            send_body_velocity(vx=BACKWARD_SPEED, duration=2.0)
        else:
            print("HOVER")
            send_body_velocity(duration=2.0)
finally:
    land_and_disarm()

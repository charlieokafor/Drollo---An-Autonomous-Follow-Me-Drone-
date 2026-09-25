from pymavlink import mavutil
import math
import time

TAKEOFF_ALTITUDE = 3.0
TARGET_DISTANCE = 3.0
DISTANCE_TOLERANCE = 0.4
CENTER_TOLERANCE = 0.12
SCAN_YAW_RATE_DEG = 18.0
TRACK_YAW_RATE_DEG = 12.0
FORWARD_SPEED = 0.5
BACKWARD_SPEED = -0.5

# WSL2/MAVProxy setup used a dedicated Python output on UDP 14552.
master = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
master.wait_heartbeat()
print("Connected to ArduPilot SITL")

def set_mode(mode):
    master.set_mode_apm(mode)
    time.sleep(0.5)

def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")

def get_altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude,
    )
    while True:
        alt = get_altitude()
        if alt is not None:
            print(f"Altitude: {alt:.2f}m")
            if alt >= altitude * 0.85:
                print("Takeoff altitude reached")
                return
        time.sleep(0.2)

def send_body_velocity(forward=0.0, right=0.0, down=0.0, yaw_rate_deg=0.0, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            1479,
            0, 0, 0,
            forward, right, down,
            0, 0, 0,
            0, math.radians(yaw_rate_deg),
        )
        time.sleep(0.1)

def stop_motion(duration=0.5):
    send_body_velocity(duration=duration)

X_SEQUENCE = [0.45, 0.30, 0.18, 0.08, 0.03, -0.05, 0.02, 0.00]
DIST_SEQUENCE = [4.8, 4.3, 3.8, 3.4, 3.0, 2.6, 2.3, 2.8, 3.2, 3.0]

def fake_person_tracking(index):
    x = X_SEQUENCE[min(index, len(X_SEQUENCE) - 1)]
    distance = DIST_SEQUENCE[min(index, len(DIST_SEQUENCE) - 1)]
    return x, distance

def scan_for_person():
    for scan_count in range(12):
        if scan_count >= 5:
            print("Person found during fake scan")
            stop_motion(1.0)
            return True
        print(f"Scan {scan_count + 1}/12")
        send_body_velocity(yaw_rate_deg=SCAN_YAW_RATE_DEG, duration=1.0)
    return False

def track_person():
    for i in range(12):
        x, distance = fake_person_tracking(i)
        yaw_rate = 0.0
        if x > CENTER_TOLERANCE:
            yaw_rate = TRACK_YAW_RATE_DEG
        elif x < -CENTER_TOLERANCE:
            yaw_rate = -TRACK_YAW_RATE_DEG

        forward = 0.0
        if distance > TARGET_DISTANCE + DISTANCE_TOLERANCE:
            forward = FORWARD_SPEED
        elif distance < TARGET_DISTANCE - DISTANCE_TOLERANCE:
            forward = BACKWARD_SPEED

        print(
            f"TRACK {i + 1:02d} | x={x:+.2f} | distance={distance:.1f}m | "
            f"vx={forward:+.2f} | yaw={yaw_rate:+.1f}deg/s"
        )
        send_body_velocity(forward=forward, yaw_rate_deg=yaw_rate, duration=1.0)

def land_and_disarm():
    stop_motion(0.5)
    set_mode("LAND")
    print("Landing...")
    time.sleep(10)
    master.arducopter_disarm()
    try:
        master.motors_disarmed_wait()
    except Exception:
        pass

try:
    set_mode("GUIDED")
    arm()
    takeoff(TAKEOFF_ALTITUDE)
    if scan_for_person():
        track_person()
    stop_motion(1.0)
finally:
    land_and_disarm()

# RECOVERED SOURCE — historical filename identified from project history: phase5_keep_center_yaw_optflow.py
# This copy preserves the recovered code verbatim below this provenance note.

# Autonomous follow-me drone — recovered project source
#
# Recovered from the archived June 2026 project code. The original standalone
# filename was not preserved in the stored artifact, so this descriptive name
# is used for the GitHub repository. The control logic below is the recovered
# source, not a newly invented replacement.
#
# EXPERIMENTAL UAV SOFTWARE: review configuration and safety conditions before use.

import depthai as dai
from pymavlink import mavutil
import time
import math

# ---------------- CONFIG ----------------

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85

# Test duration
TRACK_SECONDS = 60

# Altitude guard
ALT_DEADBAND = 0.08
ALT_KP = 0.22
ALT_MAX_VZ_NORMAL = 0.10

# Emergency altitude protection
HIGH_ALTITUDE_WARN = 1.25
HIGH_ALTITUDE_LAND = 1.45
LOW_ALTITUDE_LAND = 0.55
EMERGENCY_DESCENT_VZ = 0.25

# Yaw behavior
DEFAULT_SEARCH_YAW_RATE_DEG = 12.0
LOST_TARGET_YAW_RATE_DEG = 12.0

# If yaw direction is backwards, change this to -1
YAW_SIGN = 1

# Centering behavior
CENTER_X_TARGET = 0.50
CENTER_TOLERANCE = 0.03

# Slow proportional centering
YAW_CENTER_KP = 30.0
YAW_CENTER_MAX_DEG = 10.0
YAW_CENTER_MIN_DEG = 2.5

# Edge memory
LEFT_EDGE_LIMIT = 0.25
RIGHT_EDGE_LIMIT = 0.75

# Person detection
PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60

# Battery / failsafe messages
BATTERY_FAILSAFE_KEYWORDS = [
    "Battery Failsafe",
    "Battery 1 is low",
    "Battery 2 is low",
    "Battery critical",
    "battery failsafe",
    "battery low",
]

# ----------------------------------------

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

latest_alt = None
battery_failsafe_seen = False


def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


# Request DISTANCE_SENSOR from the MTF01 LiDAR
request_message_interval(
    mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
    200000  # 5 Hz
)

request_message_interval(
    mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
    500000
)


def update_mavlink_cache():
    global latest_alt, battery_failsafe_seen

    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()

        # Parse laser altitude (comes in as cm, convert to meters)
        if msg_type == "DISTANCE_SENSOR":
            latest_alt = msg.current_distance / 100.0

        elif msg_type == "STATUSTEXT":
            text = msg.text
            print("FC MESSAGE:", text)
            lower_text = text.lower()
            for key in BATTERY_FAILSAFE_KEYWORDS:
                if key.lower() in lower_text:
                    battery_failsafe_seen = True


def set_mode(mode):
    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print(f"Requested mode: {mode}")
    time.sleep(2)


def land_now(reason):
    print(f"LAND NOW: {reason}")
    for _ in range(10):
        send_body_velocity(0, 0, 0, 0)
        time.sleep(0.05)
    set_mode("LAND")


def wait_until_armed(timeout=20):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print("Armed:", armed)
            if armed:
                return True
    return False


def get_altitude_blocking():
    # Get laser altitude (cm to meters)
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=2)
    if not msg:
        return None
    return msg.current_distance / 100.0


def altitude_guard_vz(current_alt):
    if current_alt is None:
        return 0

    if current_alt >= HIGH_ALTITUDE_WARN:
        return EMERGENCY_DESCENT_VZ

    error = current_alt - TARGET_ALTITUDE
    if abs(error) < ALT_DEADBAND:
        return 0

    vz = error * ALT_KP
    if vz > ALT_MAX_VZ_NORMAL:
        vz = ALT_MAX_VZ_NORMAL
    if vz < -ALT_MAX_VZ_NORMAL:
        vz = -ALT_MAX_VZ_NORMAL

    return vz


def send_body_velocity(vx, vy, vz, yaw_rate_deg):
    yaw_rate_rad = math.radians(yaw_rate_deg)
    # Type mask for velocity + yaw_rate commands only (1479)
    type_mask = 1479

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0,
        yaw_rate_rad
    )


def stop_motion_with_altitude_guard():
    update_mavlink_cache()
    alt = latest_alt
    vz = altitude_guard_vz(alt)
    send_body_velocity(0, 0, vz, 0)


def yaw_rate_for_centering(center_x):
    offset = center_x - CENTER_X_TARGET
    if abs(offset) <= CENTER_TOLERANCE:
        return 0

    yaw_rate = abs(offset) * YAW_CENTER_KP
    if yaw_rate > YAW_CENTER_MAX_DEG:
        yaw_rate = YAW_CENTER_MAX_DEG
    if yaw_rate < YAW_CENTER_MIN_DEG:
        yaw_rate = YAW_CENTER_MIN_DEG

    if offset > 0:
        return YAW_SIGN * yaw_rate
    return YAW_SIGN * -yaw_rate


def search_yaw_rate_from_last_seen(last_seen_side):
    if last_seen_side == -1:
        return YAW_SIGN * -LOST_TARGET_YAW_RATE_DEG
    if last_seen_side == 1:
        return YAW_SIGN * LOST_TARGET_YAW_RATE_DEG
    return YAW_SIGN * DEFAULT_SEARCH_YAW_RATE_DEG


def fmt(value, decimals=2, suffix=""):
    if value is None:
        return "N/A"
    return f"{value:.{decimals}f}{suffix}"


# ---------- OAK-D YOLO ONLY ----------
pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()

detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera,
    dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)

labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()


# ---------- TAKEOFF ----------
print("Spoofing EKF Origin so GUIDED mode works without a real GPS...")
master.mav.set_gps_global_origin_send(
    master.target_system,
    377749000,   # Fake Lat (San Francisco)
    -1224194000, # Fake Lon
    0            # Fake Alt
)
# Give the flight controller a moment to digest the new origin
time.sleep(1)

print("Switching to standard GUIDED mode...")
set_mode("GUIDED")

print("Arming...")
master.arducopter_arm()

if not wait_until_armed():
    print("Failed to arm")
    exit()

print(f"Sending MAV_CMD_NAV_TAKEOFF command to {TARGET_ALTITUDE}m...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    TARGET_ALTITUDE
)

while True:
    update_mavlink_cache()

    if battery_failsafe_seen:
        land_now("battery failsafe during takeoff")
        exit()

    alt = get_altitude_blocking()
    if alt is not None:
        print(f"LiDAR Altitude: {alt:.2f}m")

        if alt >= HIGH_ALTITUDE_LAND:
            land_now("altitude too high during takeoff")
            exit()

        if alt >= ALTITUDE_REACHED:
            print("Altitude reached! Switching to computer vision tracking...")
            break

    time.sleep(0.2)


# ---------- KEEP PERSON CENTERED ----------
print("Starting keep-person-centered yaw test.")
print("No forward/back movement.")
print(f"Tracking time: {TRACK_SECONDS} seconds")

start_time = time.time()
last_seen_side = 0
last_center_x = None
last_seen_time = None

with pipeline:
    while pipeline.isRunning() and time.time() - start_time < TRACK_SECONDS:
        update_mavlink_cache()

        alt = latest_alt
        vz = altitude_guard_vz(alt)

        if battery_failsafe_seen:
            land_now("battery failsafe detected")
            exit()

        if alt is not None:
            if alt >= HIGH_ALTITUDE_LAND:
                land_now(f"altitude too high: {alt:.2f}m")
                exit()
            if alt <= LOW_ALTITUDE_LAND:
                land_now(f"altitude too low: {alt:.2f}m")
                exit()

        det_packet = q_det.get()
        detections = det_packet.detections
        best_person = None

        for det in detections:
            label_name = labels[det.label]
            if label_name == PERSON_LABEL and det.confidence >= CONFIDENCE_THRESHOLD:
                if best_person is None or det.confidence > best_person.confidence:
                    best_person = det

        now = time.time() - start_time

        if best_person is None:
            yaw_rate = search_yaw_rate_from_last_seen(last_seen_side)
            if last_seen_side == -1:
                search_reason = "lost target, searching LEFT"
            elif last_seen_side == 1:
                search_reason = "lost target, searching RIGHT"
            else:
                search_reason = "no target history, default search"

            print(
                f"t={now:.1f}s | alt={fmt(alt, 2, 'm')} | "
                f"vz={fmt(vz, 2)} | person=NO | "
                f"last_seen_side={last_seen_side} | "
                f"yaw_rate={yaw_rate:.1f} | {search_reason}"
            )
            send_body_velocity(0, 0, vz, yaw_rate)

        else:
            center_x = (best_person.xmin + best_person.xmax) / 2
            offset = center_x - CENTER_X_TARGET

            last_center_x = center_x
            last_seen_time = time.time()

            if center_x <= LEFT_EDGE_LIMIT:
                last_seen_side = -1
            elif center_x >= RIGHT_EDGE_LIMIT:
                last_seen_side = 1
            else:
                if offset < -CENTER_TOLERANCE:
                    last_seen_side = -1
                elif offset > CENTER_TOLERANCE:
                    last_seen_side = 1

            yaw_rate = yaw_rate_for_centering(center_x)
            if yaw_rate == 0:
                action = "CENTERED - HOLD YAW"
            else:
                if offset < 0:
                    action = "VISIBLE LEFT - SLOW CENTER LEFT"
                else:
                    action = "VISIBLE RIGHT - SLOW CENTER RIGHT"

            print(
                f"t={now:.1f}s | alt={fmt(alt, 2, 'm')} | "
                f"conf={best_person.confidence:.2f} | "
                f"center_x={center_x:.3f} | offset={offset:.3f} | "
                f"last_seen_side={last_seen_side} | "
                f"yaw_rate={yaw_rate:.1f} | {action}"
            )
            send_body_velocity(0, 0, vz, yaw_rate)

        time.sleep(0.15)


# ---------- LAND ----------
print("Tracking time complete. Switching to LAND...")
for _ in range(10):
    stop_motion_with_altitude_guard()
    time.sleep(0.05)

set_mode("LAND")
print("LAND command sent.")
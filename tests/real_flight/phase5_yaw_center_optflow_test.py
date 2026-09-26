import depthai as dai
from pymavlink import mavutil
import time
import math

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
ALT_DEADBAND = 0.08
ALT_KP = 0.22
ALT_MAX_VZ_NORMAL = 0.10
HIGH_ALTITUDE_WARN = 1.25
HIGH_ALTITUDE_LAND = 1.45
LOW_ALTITUDE_LAND = 0.55
EMERGENCY_DESCENT_VZ = 0.25
SEARCH_YAW_RATE_DEG = 12.0
YAW_SIGN = 1
CENTER_X_TARGET = 0.50
CENTER_TOLERANCE = 0.01
YAW_CENTER_KP = 35.0
YAW_CENTER_MAX_DEG = 12.0
YAW_CENTER_MIN_DEG = 3.0
CENTER_HOLD_SECONDS = 3.0
MAX_TEST_SECONDS = 75
PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60
BATTERY_FAILSAFE_KEYWORDS = [
    "Battery Failsafe", "Battery 1 is low", "Battery 2 is low",
    "Battery critical", "battery failsafe", "battery low"
]

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")
latest_alt = None
battery_failsafe_seen = False

def request_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0
    )

request_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 200000)
request_interval(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 500000)

def update_cache():
    global latest_alt, battery_failsafe_seen
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "DISTANCE_SENSOR":
            latest_alt = msg.current_distance / 100.0
        elif t == "STATUSTEXT":
            print("FC MESSAGE:", msg.text)
            low = msg.text.lower()
            if any(k.lower() in low for k in BATTERY_FAILSAFE_KEYWORDS):
                battery_failsafe_seen = True

def set_mode(name):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()[name]
    )
    time.sleep(1)

def send_body_velocity(vx, vy, vz, yaw_rate_deg):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, math.radians(yaw_rate_deg)
    )

def altitude_guard_vz(alt):
    if alt is None:
        return 0.0
    if alt >= HIGH_ALTITUDE_WARN:
        return EMERGENCY_DESCENT_VZ
    error = alt - TARGET_ALTITUDE
    if abs(error) <= ALT_DEADBAND:
        return 0.0
    return max(-ALT_MAX_VZ_NORMAL, min(ALT_MAX_VZ_NORMAL, ALT_KP * error))

def yaw_for_center(center_x):
    offset = center_x - CENTER_X_TARGET
    if abs(offset) <= CENTER_TOLERANCE:
        return 0.0
    rate = abs(offset) * YAW_CENTER_KP
    rate = min(YAW_CENTER_MAX_DEG, max(YAW_CENTER_MIN_DEG, rate))
    return YAW_SIGN * (rate if offset > 0 else -rate)

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

set_mode("GUIDED")
master.arducopter_arm()
master.motors_armed_wait(timeout=15)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
)

while True:
    update_cache()
    if battery_failsafe_seen:
        set_mode("LAND"); raise SystemExit
    if latest_alt is not None:
        print(f"LiDAR altitude: {latest_alt:.2f}m")
        if latest_alt >= HIGH_ALTITUDE_LAND:
            set_mode("LAND"); raise SystemExit
        if latest_alt >= ALTITUDE_REACHED:
            break
    time.sleep(0.2)

start = time.time()
centered_since = None
with pipeline:
    while pipeline.isRunning() and time.time() - start < MAX_TEST_SECONDS:
        update_cache()
        if battery_failsafe_seen:
            break
        alt = latest_alt
        if alt is not None and (alt >= HIGH_ALTITUDE_LAND or alt <= LOW_ALTITUDE_LAND):
            break
        vz = altitude_guard_vz(alt)
        people = [
            d for d in q_det.get().detections
            if labels[d.label] == PERSON_LABEL and d.confidence >= CONFIDENCE_THRESHOLD
        ]
        if not people:
            centered_since = None
            send_body_velocity(0, 0, vz, SEARCH_YAW_RATE_DEG)
            continue
        p = max(people, key=lambda d: d.confidence)
        center_x = (p.xmin + p.xmax) / 2.0
        yaw = yaw_for_center(center_x)
        print(f"alt={alt} conf={p.confidence:.2f} center_x={center_x:.3f} yaw={yaw:.1f}")
        send_body_velocity(0, 0, vz, yaw)
        if yaw == 0:
            if centered_since is None:
                centered_since = time.time()
            elif time.time() - centered_since >= CENTER_HOLD_SECONDS:
                break
        else:
            centered_since = None
        time.sleep(0.15)

for _ in range(10):
    send_body_velocity(0, 0, altitude_guard_vz(latest_alt), 0)
    time.sleep(0.05)
set_mode("LAND")

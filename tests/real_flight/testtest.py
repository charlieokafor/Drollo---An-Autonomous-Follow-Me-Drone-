import depthai as dai
from pymavlink import mavutil
import time
import math

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
TRACK_SECONDS = 60
ALT_DEADBAND = 0.08
ALT_KP = 0.22
ALT_MAX_VZ_NORMAL = 0.10
HIGH_ALTITUDE_WARN = 1.25
HIGH_ALTITUDE_LAND = 1.45
LOW_ALTITUDE_LAND = 0.55
EMERGENCY_DESCENT_VZ = 0.25
DEFAULT_SEARCH_YAW_RATE_DEG = 12.0
LOST_TARGET_YAW_RATE_DEG = 12.0
CENTER_X_TARGET = 0.50
CENTER_TOLERANCE = 0.03
YAW_CENTER_KP = 30.0
YAW_CENTER_MAX_DEG = 10.0
YAW_CENTER_MIN_DEG = 2.5
LEFT_EDGE_LIMIT = 0.25
RIGHT_EDGE_LIMIT = 0.75
PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")
latest_alt = None

def request_interval(msg_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, interval_us, 0, 0, 0, 0, 0
    )
request_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 200000)
request_interval(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 500000)

def update_altitude():
    global latest_alt
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "DISTANCE_SENSOR":
            latest_alt = msg.current_distance / 100.0
        elif msg.get_type() == "STATUSTEXT":
            print("FC MESSAGE:", msg.text)

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
    return max(-ALT_MAX_VZ_NORMAL, min(ALT_MAX_VZ_NORMAL, error * ALT_KP))

def center_yaw(cx):
    error = cx - CENTER_X_TARGET
    if abs(error) <= CENTER_TOLERANCE:
        return 0.0
    r = min(YAW_CENTER_MAX_DEG, max(YAW_CENTER_MIN_DEG, abs(error) * YAW_CENTER_KP))
    return r if error > 0 else -r

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

set_mode("GUIDED_NOGPS")
master.arducopter_arm()
master.motors_armed_wait(timeout=15)
print("GUIDED_NOGPS armed. Beginning velocity climb.")

takeoff_start = time.time()
while time.time() - takeoff_start < 12:
    update_altitude()
    if latest_alt is not None:
        print(f"LiDAR Altitude: {latest_alt:.2f}m")
        if latest_alt >= ALTITUDE_REACHED:
            break
    send_body_velocity(0, 0, -0.25, 0)
    time.sleep(0.1)

# stop climb before vision tracking
for _ in range(5):
    update_altitude()
    send_body_velocity(0, 0, altitude_guard_vz(latest_alt), 0)
    time.sleep(0.1)

start = time.time()
last_seen_side = 0
with pipeline:
    while pipeline.isRunning() and time.time() - start < TRACK_SECONDS:
        update_altitude()
        alt = latest_alt
        if alt is not None and (alt >= HIGH_ALTITUDE_LAND or alt <= LOW_ALTITUDE_LAND):
            print("Altitude safety limit. Landing.")
            break
        vz = altitude_guard_vz(alt)
        people = [
            d for d in q_det.get().detections
            if labels[d.label] == PERSON_LABEL and d.confidence >= CONFIDENCE_THRESHOLD
        ]
        if not people:
            yaw = -LOST_TARGET_YAW_RATE_DEG if last_seen_side < 0 else LOST_TARGET_YAW_RATE_DEG
            send_body_velocity(0, 0, vz, yaw)
            time.sleep(0.15)
            continue
        p = max(people, key=lambda d: d.confidence)
        cx = (p.xmin + p.xmax) / 2.0
        if cx <= LEFT_EDGE_LIMIT:
            last_seen_side = -1
        elif cx >= RIGHT_EDGE_LIMIT:
            last_seen_side = 1
        elif cx < CENTER_X_TARGET - CENTER_TOLERANCE:
            last_seen_side = -1
        elif cx > CENTER_X_TARGET + CENTER_TOLERANCE:
            last_seen_side = 1
        yaw = center_yaw(cx)
        print(f"alt={alt} center_x={cx:.3f} yaw={yaw:.1f}")
        send_body_velocity(0, 0, vz, yaw)
        time.sleep(0.15)

set_mode("LAND")

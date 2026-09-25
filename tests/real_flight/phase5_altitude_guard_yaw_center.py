import depthai as dai
from pymavlink import mavutil
import math
import time

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
ALTITUDE_BAND = 0.12
VERTICAL_CORRECTION_SPEED = 0.18
MAX_SAFE_ALTITUDE = 1.80
MIN_SAFE_ALTITUDE = 0.35

PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60
SEARCH_YAW_RATE_DEG = 12.0
CORRECTION_YAW_RATE_DEG = 6.0  # later slowed to 4 deg/s after circling behavior
CENTER_TARGET = 0.50
CENTER_TOLERANCE = 0.01
CENTER_HOLD_SECONDS = 3
MAX_TEST_SECONDS = 70

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()

def get_altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def altitude_vz(alt):
    if alt is None:
        return 0.0
    if alt > TARGET_ALTITUDE + ALTITUDE_BAND:
        return VERTICAL_CORRECTION_SPEED   # BODY_NED +Z = down
    if alt < TARGET_ALTITUDE - ALTITUDE_BAND:
        return -VERTICAL_CORRECTION_SPEED  # up
    return 0.0

def send_body_velocity(vx, vy, vz, yaw_rate_deg):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        1479,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, math.radians(yaw_rate_deg),
    )

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

master.set_mode_apm("GUIDED")
master.arducopter_arm()
master.motors_armed_wait()
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE,
)

while True:
    alt = get_altitude()
    if alt is not None and alt >= ALTITUDE_REACHED:
        break

start = time.time()
centered_since = None
with pipeline:
    while pipeline.isRunning() and time.time() - start < MAX_TEST_SECONDS:
        alt = get_altitude()
        if alt is not None and (alt > MAX_SAFE_ALTITUDE or alt < MIN_SAFE_ALTITUDE):
            print(f"Unsafe altitude {alt:.2f}m — LAND")
            break
        vz = altitude_vz(alt)

        packet = q_det.get()
        people = [d for d in packet.detections if labels[d.label] == PERSON_LABEL and d.confidence >= CONFIDENCE_THRESHOLD]
        if not people:
            centered_since = None
            yaw = SEARCH_YAW_RATE_DEG
            action = "SEARCH"
        else:
            person = max(people, key=lambda d: d.confidence)
            cx = (person.xmin + person.xmax) / 2
            offset = cx - CENTER_TARGET
            if abs(offset) <= CENTER_TOLERANCE:
                yaw = 0.0
                action = "CENTERED"
                if centered_since is None:
                    centered_since = time.time()
            else:
                centered_since = None
                # Historical initial sign mapping. It was later swapped after test logs.
                yaw = CORRECTION_YAW_RATE_DEG if offset < 0 else -CORRECTION_YAW_RATE_DEG
                action = "CENTER LEFT" if offset < 0 else "CENTER RIGHT"

        print(f"alt={alt} vz={vz:+.2f} yaw={yaw:+.1f} | {action}")
        send_body_velocity(0, 0, vz, yaw)

        if centered_since and time.time() - centered_since >= CENTER_HOLD_SECONDS:
            print("Center hold complete")
            break
        time.sleep(0.1)

send_body_velocity(0, 0, 0, 0)
master.set_mode_apm("LAND")

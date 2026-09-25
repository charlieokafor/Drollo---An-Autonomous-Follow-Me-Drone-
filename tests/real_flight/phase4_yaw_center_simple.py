import depthai as dai
from pymavlink import mavutil
import math
import time

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
SEARCH_YAW_RATE_DEG = 12.0
CENTER_TARGET = 0.50
CENTER_TOLERANCE = 0.01  # began at 0.08; tightened after the first successful test
CENTER_HOLD_SECONDS = 3
MAX_TEST_SECONDS = 60
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()

def send_yaw(yaw_rate_deg):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, math.radians(yaw_rate_deg),
    )

def altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

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
    alt = altitude()
    if alt is not None and alt >= ALTITUDE_REACHED:
        break

start = time.time()
centered_since = None
with pipeline:
    while pipeline.isRunning() and time.time() - start < MAX_TEST_SECONDS:
        packet = q_det.get()
        people = [
            d for d in packet.detections
            if labels[d.label] == "person" and d.confidence >= CONFIDENCE_THRESHOLD
        ]
        if not people:
            centered_since = None
            print("NO PERSON | SEARCH")
            send_yaw(SEARCH_YAW_RATE_DEG)
            time.sleep(0.1)
            continue

        person = max(people, key=lambda d: d.confidence)
        center_x = (person.xmin + person.xmax) / 2
        offset = center_x - CENTER_TARGET
        print(f"PERSON | center_x={center_x:.3f} offset={offset:+.3f}")

        # The deliberately simple Phase-4 behavior kept yawing in one direction
        # until the person reached the center band.
        if abs(offset) > CENTER_TOLERANCE:
            centered_since = None
            send_yaw(SEARCH_YAW_RATE_DEG)
        else:
            send_yaw(0)
            if centered_since is None:
                centered_since = time.time()
            if time.time() - centered_since >= CENTER_HOLD_SECONDS:
                print("Centered hold complete")
                break
        time.sleep(0.1)

send_yaw(0)
master.set_mode_apm("LAND")

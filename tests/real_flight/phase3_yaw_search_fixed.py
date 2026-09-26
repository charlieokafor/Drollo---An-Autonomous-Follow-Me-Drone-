# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and phase purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify configuration and test safely.

import depthai as dai
from pymavlink import mavutil
import time
import math

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
SEARCH_YAW_RATE_DEG = 12.0  # original test began at 3 deg/s, then was tuned to 12
PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")

def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )

def altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def send_yaw_rate(yaw_rate_deg):
    # Fix that mattered in this phase: SET_POSITION_TARGET yaw_rate is radians/s.
    yaw_rate_rad = math.radians(yaw_rate_deg)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, yaw_rate_rad,
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

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000)
master.set_mode_apm("GUIDED")
time.sleep(1)
master.arducopter_arm()
master.motors_armed_wait()

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE,
)

while True:
    alt = altitude()
    if alt is not None:
        print(f"Altitude: {alt:.2f}m")
        if alt >= ALTITUDE_REACHED:
            break

print(f"Searching at {SEARCH_YAW_RATE_DEG:.1f} deg/s")
with pipeline:
    while pipeline.isRunning():
        packet = q_det.get()
        person = None
        for det in packet.detections:
            if labels[det.label] == PERSON_LABEL and det.confidence >= CONFIDENCE_THRESHOLD:
                person = det
                break
        if person is not None:
            print("Person detected — stop yaw")
            send_yaw_rate(0)
            time.sleep(3)
            break
        print("No person — yaw search")
        send_yaw_rate(SEARCH_YAW_RATE_DEG)
        time.sleep(0.1)

send_yaw_rate(0)
master.set_mode_apm("LAND")
print("LAND")

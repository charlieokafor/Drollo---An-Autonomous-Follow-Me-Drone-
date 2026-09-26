# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify port, mode, parameters and safety before use.

import depthai as dai
from pymavlink import mavutil
import time

PORT = "/dev/serial0"
BAUD = 115200
CONFIDENCE_THRESHOLD = 0.60
DIAGNOSTIC_SECONDS = 12

master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()
print("Connected to FC")

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

def is_armed(msg):
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def diagnose(seconds):
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == "HEARTBEAT":
                print(f"HEARTBEAT: mode={master.flightmode} armed={is_armed(msg)}")
            elif msg.get_type() == "STATUSTEXT":
                print("FC MESSAGE:", msg.text)
            elif msg.get_type() == "COMMAND_ACK":
                print(f"COMMAND_ACK: command={msg.command} result={msg.result}")
        time.sleep(0.05)

with pipeline:
    while pipeline.isRunning():
        packet = q_det.get()
        person = any(
            labels[d.label] == "person" and d.confidence >= CONFIDENCE_THRESHOLD
            for d in packet.detections
        )
        if not person:
            print("No person")
            continue

        print("Person detected — requesting GUIDED and ARM")
        master.set_mode_apm("GUIDED")
        time.sleep(1)
        master.arducopter_arm()
        diagnose(DIAGNOSTIC_SECONDS)
        master.arducopter_disarm()
        print("Safety DISARM sent")
        break

import depthai as dai
from pymavlink import mavutil
import time

PORT = "/dev/serial0"
BAUD = 115200
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

def listen(seconds=8):
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == "STATUSTEXT":
                print("FC MESSAGE:", msg.text)
            elif msg.get_type() == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print(f"HEARTBEAT: mode={master.flightmode} armed={armed}")
        time.sleep(0.05)

with pipeline:
    while pipeline.isRunning():
        packet = q_det.get()
        if any(labels[d.label] == "person" and d.confidence >= CONFIDENCE_THRESHOLD for d in packet.detections):
            print("Person detected — STABILIZE arm test")
            master.set_mode_apm("STABILIZE")
            time.sleep(1)
            master.arducopter_arm()
            listen()
            master.arducopter_disarm()
            break

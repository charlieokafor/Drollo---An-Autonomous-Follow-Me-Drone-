import depthai as dai
from pymavlink import mavutil
import time

PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to flight controller")

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

print("Waiting for a person...")
with pipeline:
    while pipeline.isRunning():
        packet = q_det.get()
        person = None
        for det in packet.detections:
            if labels[det.label] == PERSON_LABEL and det.confidence >= CONFIDENCE_THRESHOLD:
                person = det
                break
        if person is None:
            print("No person")
            continue

        print(f"Person detected: {person.confidence:.2f}")
        mode_id = master.mode_mapping()["GUIDED"]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(2)

        print("Arming...")
        master.arducopter_arm()
        for _ in range(8):
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("ARMED")
                break
        else:
            print("Arm failed")
            break

        time.sleep(5)
        print("Disarming...")
        master.arducopter_disarm()
        for _ in range(10):
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("DISARMED")
                break
        break

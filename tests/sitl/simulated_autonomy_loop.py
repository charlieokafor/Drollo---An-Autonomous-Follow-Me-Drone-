# RESTORED PROJECT TEST ARTIFACT
# This file was restored from the project's original ChatGPT code history.
# Constants, filenames, phase purpose and control structure are preserved wherever
# the history retained them. Where the original source bytes were unavailable,
# glue/boilerplate was reconstructed to make the historical test a standalone file.
# EXPERIMENTAL UAV SOFTWARE — review configuration and test safely.

import depthai as dai
from pymavlink import mavutil
import time

CONFIDENCE_THRESHOLD = 0.60
LEFT_LIMIT = 0.40
RIGHT_LIMIT = 0.60
FAR_AREA = 0.08
CLOSE_AREA = 0.35

master = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
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

with pipeline:
    while pipeline.isRunning():
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        mode = master.flightmode if hb else "UNKNOWN"
        armed = master.motors_armed()

        packet = q_det.get()
        best = None
        for det in packet.detections:
            if labels[det.label] == "person" and det.confidence >= CONFIDENCE_THRESHOLD:
                if best is None or det.confidence > best.confidence:
                    best = det

        if best is None:
            print(f"[AUTO] mode={mode} armed={armed} | Person NO | SEARCH/YAW LATER")
        else:
            center_x = (best.xmin + best.xmax) / 2.0
            area = (best.xmax - best.xmin) * (best.ymax - best.ymin)
            if center_x < LEFT_LIMIT:
                yaw_action = "TURN LEFT"
            elif center_x > RIGHT_LIMIT:
                yaw_action = "TURN RIGHT"
            else:
                yaw_action = "CENTERED"

            if area < FAR_AREA:
                distance_action = "TOO FAR / MOVE FORWARD"
            elif area > CLOSE_AREA:
                distance_action = "TOO CLOSE / MOVE BACK"
            else:
                distance_action = "GOOD DISTANCE"

            print(
                f"[AUTO] mode={mode} armed={armed} | Person YES | "
                f"x={center_x:.2f} area={area:.3f} | {yaw_action} | {distance_action}"
            )
        time.sleep(0.1)

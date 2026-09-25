import depthai as dai
import time

CONFIDENCE_THRESHOLD = 0.60
LEFT_LIMIT = 0.40
RIGHT_LIMIT = 0.60
FAR_AREA = 0.08
CLOSE_AREA = 0.35

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
        packet = q_det.get()
        people = [
            d for d in packet.detections
            if labels[d.label] == "person" and d.confidence >= CONFIDENCE_THRESHOLD
        ]
        if not people:
            print("Person NO | SEARCH/YAW LATER")
            continue

        person = max(people, key=lambda d: d.confidence)
        center_x = (person.xmin + person.xmax) / 2.0
        width = person.xmax - person.xmin
        height = person.ymax - person.ymin
        area = width * height

        if center_x < LEFT_LIMIT:
            yaw_action = "TURN LEFT"
        elif center_x > RIGHT_LIMIT:
            yaw_action = "TURN RIGHT"
        else:
            yaw_action = "CENTERED"

        if area < FAR_AREA:
            range_action = "TOO FAR / MOVE FORWARD"
        elif area > CLOSE_AREA:
            range_action = "TOO CLOSE / MOVE BACK"
        else:
            range_action = "GOOD DISTANCE"

        print(
            f"Person YES | conf={person.confidence:.2f} | x={center_x:.2f} | "
            f"area={area:.3f} | {yaw_action} | {range_action}"
        )
        time.sleep(0.05)

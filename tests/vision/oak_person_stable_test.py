import depthai as dai
import blobconverter
import numpy as np
from collections import deque

CENTER_TOLERANCE = 0.10
TARGET_DISTANCE_M = 3.0
DISTANCE_TOLERANCE_M = 0.4
MIN_VALID_DISTANCE_M = 0.4
MAX_VALID_DISTANCE_M = 8.0
MAX_DISTANCE_JUMP_M = 1.5
X_HISTORY = 5
DISTANCE_HISTORY = 7
PERSON_LABEL = 15
CONFIDENCE_THRESHOLD = 0.50

x_hist = deque(maxlen=X_HISTORY)
dist_hist = deque(maxlen=DISTANCE_HISTORY)
last_good_distance = None

pipeline = dai.Pipeline()

rgb = pipeline.create(dai.node.ColorCamera)
rgb.setPreviewSize(300, 300)
rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
rgb.setInterleaved(False)
rgb.setFps(30)
rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setSubpixel(True)
left.out.link(stereo.left)
right.out.link(stereo.right)

nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
nn.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
nn.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6))
rgb.preview.link(nn.input)

x_nn = pipeline.create(dai.node.XLinkOut)
x_nn.setStreamName("nn")
nn.out.link(x_nn.input)

x_depth = pipeline.create(dai.node.XLinkOut)
x_depth.setStreamName("depth")
stereo.depth.link(x_depth.input)

def robust_depth(depth_mm, det):
    global last_good_distance
    h, w = depth_mm.shape
    # Torso-center ROI rather than using the whole box/background.
    x1 = int(max(0, min(1, det.xmin + (det.xmax-det.xmin)*0.30)) * w)
    x2 = int(max(0, min(1, det.xmax - (det.xmax-det.xmin)*0.30)) * w)
    y1 = int(max(0, min(1, det.ymin + (det.ymax-det.ymin)*0.25)) * h)
    y2 = int(max(0, min(1, det.ymax - (det.ymax-det.ymin)*0.20)) * h)
    roi = depth_mm[y1:y2, x1:x2]
    values = roi[(roi >= MIN_VALID_DISTANCE_M*1000) & (roi <= MAX_VALID_DISTANCE_M*1000)]
    if values.size == 0:
        return last_good_distance
    d = float(np.median(values)) / 1000.0
    if last_good_distance is not None and abs(d - last_good_distance) > MAX_DISTANCE_JUMP_M:
        return last_good_distance
    dist_hist.append(d)
    last_good_distance = float(np.median(dist_hist))
    return last_good_distance

with dai.Device(pipeline) as device:
    q_nn = device.getOutputQueue("nn", maxSize=4, blocking=True)
    q_depth = device.getOutputQueue("depth", maxSize=4, blocking=True)
    while True:
        detections = q_nn.get().detections
        depth = q_depth.get().getFrame()
        people = [d for d in detections if d.label == PERSON_LABEL]
        if not people:
            print("NO PERSON")
            continue

        person = max(people, key=lambda d: d.confidence)
        x = ((person.xmin + person.xmax) / 2.0) - 0.5
        x_hist.append(x)
        smooth_x = float(np.median(x_hist))
        distance = robust_depth(depth, person)

        if smooth_x < -CENTER_TOLERANCE:
            center_action = "PERSON LEFT"
        elif smooth_x > CENTER_TOLERANCE:
            center_action = "PERSON RIGHT"
        else:
            center_action = "CENTERED"

        if distance is None:
            distance_action = "DISTANCE UNKNOWN"
            distance_text = "unknown"
        elif distance > TARGET_DISTANCE_M + DISTANCE_TOLERANCE_M:
            distance_action = "TOO FAR"
            distance_text = f"{distance:.2f}m"
        elif distance < TARGET_DISTANCE_M - DISTANCE_TOLERANCE_M:
            distance_action = "TOO CLOSE"
            distance_text = f"{distance:.2f}m"
        else:
            distance_action = "GOOD DISTANCE"
            distance_text = f"{distance:.2f}m"

        print(f"TRACKING | x: {smooth_x:+.2f} | distance: {distance_text} | {center_action} | {distance_action}")

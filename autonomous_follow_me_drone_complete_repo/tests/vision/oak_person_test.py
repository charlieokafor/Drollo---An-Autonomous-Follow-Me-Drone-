# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and phase purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review before use.

# Earlier OAK-D person/depth validation restored from its preserved console output.
# The historical run printed: "Person detected | x_offset: ... | distance: ...m".

import depthai as dai
import blobconverter
import numpy as np

PERSON_LABEL = 15
CONFIDENCE_THRESHOLD = 0.50
MIN_DEPTH_MM = 300
MAX_DEPTH_MM = 15000

pipeline = dai.Pipeline()
rgb = pipeline.create(dai.node.ColorCamera)
rgb.setPreviewSize(300, 300)
rgb.setInterleaved(False)
rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
left.out.link(stereo.left)
right.out.link(stereo.right)

nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
nn.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
nn.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6))
rgb.preview.link(nn.input)

x_nn = pipeline.create(dai.node.XLinkOut); x_nn.setStreamName("nn"); nn.out.link(x_nn.input)
x_depth = pipeline.create(dai.node.XLinkOut); x_depth.setStreamName("depth"); stereo.depth.link(x_depth.input)

with dai.Device(pipeline) as device:
    q_nn = device.getOutputQueue("nn", 4, True)
    q_depth = device.getOutputQueue("depth", 4, True)
    while True:
        detections = q_nn.get().detections
        depth = q_depth.get().getFrame()
        people = [d for d in detections if d.label == PERSON_LABEL]
        if not people:
            print("No person")
            continue
        det = max(people, key=lambda d: d.confidence)
        center = (det.xmin + det.xmax) / 2.0
        x_offset = center - 0.5
        h, w = depth.shape
        cx, cy = int(center*w), int(((det.ymin+det.ymax)/2.0)*h)
        r = 10
        roi = depth[max(0,cy-r):min(h,cy+r), max(0,cx-r):min(w,cx+r)]
        valid = roi[(roi >= MIN_DEPTH_MM) & (roi <= MAX_DEPTH_MM)]
        if valid.size:
            distance = float(np.median(valid))/1000.0
            print(f"Person detected | x_offset: {x_offset:.2f} | distance: {distance:.2f}m")
        else:
            print(f"Person detected | x_offset: {x_offset:.2f} | distance: unknown")

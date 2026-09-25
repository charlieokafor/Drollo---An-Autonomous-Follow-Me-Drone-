import depthai as dai
import numpy as np

MIN_DEPTH_MM = 300
MAX_DEPTH_MM = 8000
EMERGENCY_STOP_MM = 1000
AVOID_DISTANCE_MM = 2500
CLEAR_DISTANCE_MM = 3500

def percentile_depth(region, percentile=10):
    valid = region[(region >= MIN_DEPTH_MM) & (region <= MAX_DEPTH_MM)]
    if valid.size == 0:
        return None
    return float(np.percentile(valid, percentile))

def build_depth_grid(depth):
    h, w = depth.shape
    third = w // 3
    return {
        "LEFT": percentile_depth(depth[:, :third]),
        "CENTER": percentile_depth(depth[:, third:2*third]),
        "RIGHT": percentile_depth(depth[:, 2*third:]),
    }

def decide_direction(grid):
    center = grid["CENTER"]
    if center is not None and center <= EMERGENCY_STOP_MM:
        return "STOP"
    if center is None or center > CLEAR_DISTANCE_MM:
        return "FORWARD_OK"
    left = grid["LEFT"] or 0
    right = grid["RIGHT"] or 0
    if max(left, right) < AVOID_DISTANCE_MM:
        return "STOP"
    return "TURN_LEFT" if left > right else "TURN_RIGHT"

pipeline = dai.Pipeline()
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)
left.out.link(stereo.left)
right.out.link(stereo.right)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("depth")
stereo.depth.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("depth", maxSize=4, blocking=False)
    while True:
        depth = q.get().getFrame()
        grid = build_depth_grid(depth)
        action = decide_direction(grid)
        print(grid, "->", action)

# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and staged-test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

import depthai as dai
import numpy as np
from collections import deque
import time

TARGET_DISTANCE_M=5.5
DISTANCE_DEADBAND_M=0.4
DISTANCE_KP=0.12
MAX_FORWARD_SPEED=0.25
DEPTH_HISTORY=5
TEST_SECONDS=35

# Safety regression after the first continuous-motion test: calculate the exact
# requested vx but DO NOT transmit any horizontal movement command.
pipeline=dai.Pipeline();left=pipeline.create(dai.node.MonoCamera);right=pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B);right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
stereo=pipeline.create(dai.node.StereoDepth);left.out.link(stereo.left);right.out.link(stereo.right)
xo=pipeline.create(dai.node.XLinkOut);xo.setStreamName('depth');stereo.depth.link(xo.input)

hist=deque(maxlen=DEPTH_HISTORY)
with dai.Device(pipeline) as dev:
    q=dev.getOutputQueue('depth',4,True);start=time.time()
    while time.time()-start<TEST_SECONDS:
        f=q.get().getFrame();h,w=f.shape;roi=f[int(h*.35):int(h*.70),int(w*.40):int(w*.60)];v=roi[(roi>=1000)&(roi<=9000)]
        if v.size:hist.append(float(np.median(v))/1000)
        d=float(np.median(hist)) if hist else None
        requested=0.0
        if d is not None:
            e=d-TARGET_DISTANCE_M
            if abs(e)>DISTANCE_DEADBAND_M:requested=max(-MAX_FORWARD_SPEED,min(MAX_FORWARD_SPEED,e*DISTANCE_KP))
        action='HOLD' if requested==0 else ('WOULD MOVE FORWARD' if requested>0 else 'WOULD MOVE BACK')
        print(f'distance={d} requested_vx={requested:+.2f} | {action} | SENT vx=0.00')
        time.sleep(.2)

# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and phase purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review before use.

# Split resource test created after the simultaneous YOLO+StereoDepth SHAVE failure.

import csv
import depthai as dai
import numpy as np
from pymavlink import mavutil
import time

BBOX_TEST_SECONDS=45
DEPTH_TEST_SECONDS=45
CONFIDENCE_THRESHOLD=0.60
CSV_FILE="phase6_distance_split_log.csv"

master=mavutil.mavlink_connection("/dev/serial0",baud=115200)
master.wait_heartbeat()

with open(CSV_FILE,"w",newline="") as f:
    writer=csv.writer(f)
    writer.writerow(["phase","time_sec","altitude_m","confidence","center_x","box_width","box_height","box_area","depth_m"])

    # Phase A: YOLO bounding boxes only.
    p=dai.Pipeline(); cam=p.create(dai.node.Camera).build(); det=p.create(dai.node.DetectionNetwork).build(cam,dai.NNModelDescription("yolov6-nano")); det.setConfidenceThreshold(CONFIDENCE_THRESHOLD); labels=det.getClasses(); q=det.out.createOutputQueue(); p.start()
    start=time.time()
    with p:
        while p.isRunning() and time.time()-start<BBOX_TEST_SECONDS:
            pkt=q.get(); people=[d for d in pkt.detections if labels[d.label]=="person" and d.confidence>=CONFIDENCE_THRESHOLD]
            if not people:continue
            d=max(people,key=lambda x:x.confidence); cx=(d.xmin+d.xmax)/2; bw=d.xmax-d.xmin; bh=d.ymax-d.ymin; area=bw*bh
            writer.writerow(["bbox",time.time()-start,"",d.confidence,cx,bw,bh,area,""])
            f.flush()

    # Phase B: stereo depth only. No YOLO blob is loaded, freeing NN resources.
    p2=dai.Pipeline()
    left=p2.create(dai.node.MonoCamera); right=p2.create(dai.node.MonoCamera)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P); right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B); right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    stereo=p2.create(dai.node.StereoDepth); left.out.link(stereo.left); right.out.link(stereo.right)
    xo=p2.create(dai.node.XLinkOut); xo.setStreamName("depth"); stereo.depth.link(xo.input)
    with dai.Device(p2) as dev:
        qd=dev.getOutputQueue("depth",4,True); start=time.time()
        while time.time()-start<DEPTH_TEST_SECONDS:
            frame=qd.get().getFrame(); h,w=frame.shape; roi=frame[h//3:2*h//3,w//3:2*w//3]; valid=roi[roi>0]
            depth_m=float(np.median(valid))/1000.0 if valid.size else None
            writer.writerow(["depth",time.time()-start,"","","","","","",depth_m]); f.flush()

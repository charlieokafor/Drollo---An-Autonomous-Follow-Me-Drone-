import csv
import depthai as dai
import numpy as np
from pymavlink import mavutil
import time

TARGET_ALTITUDE=1.0
ALTITUDE_REACHED=0.85
ALTITUDE_BAND=0.12
VERTICAL_CORRECTION_SPEED=0.18
CONFIDENCE_THRESHOLD=0.60
TEST_SECONDS=90
CSV_FILE="phase6_distance_log.csv"

master=mavutil.mavlink_connection("/dev/serial0",baud=115200)
master.wait_heartbeat()

def altitude():
    m=master.recv_match(type="GLOBAL_POSITION_INT",blocking=True,timeout=2)
    return None if m is None else m.relative_alt/1000.0

def send_vz(vz):
    master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,0,0,vz,0,0,0,0,0)

def guard(alt):
    if alt is None:return 0
    if alt>1.12:return 0.18
    if alt<0.88:return -0.18
    return 0

pipeline=dai.Pipeline()
cam=pipeline.create(dai.node.Camera).build()
det=pipeline.create(dai.node.DetectionNetwork).build(cam,dai.NNModelDescription("yolov6-nano"))
det.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels=det.getClasses(); q_det=det.out.createOutputQueue()

left=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right=pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo=pipeline.create(dai.node.StereoDepth)
# This is intentionally representative of the simultaneous network+stereo experiment.
# API details varied with the DepthAI build used on the prototype.
q_depth=stereo.depth.createOutputQueue()
pipeline.start()

master.set_mode_apm("GUIDED"); master.arducopter_arm(); master.motors_armed_wait()
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE)
while True:
    a=altitude()
    if a is not None and a>=ALTITUDE_REACHED:break

with open(CSV_FILE,"w",newline="") as f,pipeline:
    w=csv.writer(f); w.writerow(["time_sec","altitude_m","confidence","center_x","box_width","box_height","box_area","depth_m"])
    start=time.time()
    while pipeline.isRunning() and time.time()-start<TEST_SECONDS:
        a=altitude(); send_vz(guard(a))
        pkt=q_det.get(); depth=q_depth.get().getFrame()
        people=[d for d in pkt.detections if labels[d.label]=="person" and d.confidence>=CONFIDENCE_THRESHOLD]
        if not people:continue
        d=max(people,key=lambda x:x.confidence); cx=(d.xmin+d.xmax)/2; bw=d.xmax-d.xmin; bh=d.ymax-d.ymin; area=bw*bh
        h,ww=depth.shape; px=int(cx*ww); py=int(((d.ymin+d.ymax)/2)*h); roi=depth[max(0,py-10):py+10,max(0,px-10):px+10]; valid=roi[roi>0]
        depth_m=float(np.median(valid))/1000.0 if valid.size else None
        w.writerow([time.time()-start,a,d.confidence,cx,bw,bh,area,depth_m])

send_vz(0); master.set_mode_apm("LAND")

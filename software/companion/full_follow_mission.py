# This file intentionally combines the separately tested yaw-centering and
# depth-follow logic into one readable end-to-end state machine. It is a
# combined test for the engineering archive, not claimed as an exact original.
from pymavlink import mavutil
import depthai as dai
import time,math

TARGET_ALTITUDE=1.0; ALTITUDE_REACHED=0.85
TARGET_DISTANCE_M=5.5; DISTANCE_DEADBAND_M=0.25; DISTANCE_KP=0.17; MAX_FORWARD_SPEED=0.38
CENTER_TARGET=0.50; CENTER_TOLERANCE=0.03; YAW_KP=30.0; YAW_MAX=10.0; YAW_MIN=2.5; SEARCH_YAW=12.0
TRACK_SECONDS=60; CONFIDENCE_THRESHOLD=0.60
master=mavutil.mavlink_connection("/dev/serial0",baud=115200);master.wait_heartbeat()

def mode(name):
    master.mav.set_mode_send(master.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,master.mode_mapping()[name]);time.sleep(1)
def cmd(vx=0,vz=0,yaw=0):
    master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,vx,0,vz,0,0,0,0,math.radians(yaw))
def depth_for_person(det):
    # Original project obtained person depth from OAK-D stereo data.
    return None
def yaw_for(cx):
    e=cx-CENTER_TARGET
    if abs(e)<=CENTER_TOLERANCE:return 0
    r=max(YAW_MIN,min(YAW_MAX,abs(e)*YAW_KP));return r if e>0 else -r

mode("GUIDED");master.arducopter_arm();master.motors_armed_wait(timeout=15)
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE);time.sleep(4)
pipeline=dai.Pipeline();cam=pipeline.create(dai.node.Camera).build();detnet=pipeline.create(dai.node.DetectionNetwork).build(cam,dai.NNModelDescription("yolov6-nano"));detnet.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels=detnet.getClasses();q=detnet.out.createOutputQueue();pipeline.start();last_side=0;start=time.time()
try:
    with pipeline:
        while pipeline.isRunning() and time.time()-start<TRACK_SECONDS:
            people=[d for d in q.get().detections if labels[d.label]=="person" and d.confidence>=CONFIDENCE_THRESHOLD]
            if not people:
                cmd(0,0,-SEARCH_YAW if last_side<0 else SEARCH_YAW);continue
            p=max(people,key=lambda d:d.confidence);cx=(p.xmin+p.xmax)/2;last_side=-1 if cx<CENTER_TARGET else 1
            yaw=yaw_for(cx);depth=depth_for_person(p);vx=0
            if depth is not None:
                e=depth-TARGET_DISTANCE_M
                if abs(e)>DISTANCE_DEADBAND_M:vx=max(-MAX_FORWARD_SPEED,min(MAX_FORWARD_SPEED,e*DISTANCE_KP))
            print(f"center_x={cx:.3f} yaw={yaw:.1f} depth={depth} vx={vx:.2f}")
            cmd(vx,0,yaw);time.sleep(.15)
finally:
    cmd();mode("LAND")

import depthai as dai
import numpy as np
from collections import deque
from pymavlink import mavutil
import time

TARGET_ALTITUDE=1.0
ALTITUDE_REACHED=0.85
ALT_DEADBAND=0.08
ALT_KP=0.22
ALT_MAX_VZ=0.10
MIN_SAFE_ALTITUDE=0.35
MAX_SAFE_ALTITUDE=1.80
TARGET_DISTANCE_M=5.5
DISTANCE_DEADBAND_M=0.4
DISTANCE_KP=0.12
MAX_FORWARD_SPEED=0.25
TEST_SECONDS=35
DEPTH_HISTORY=5

master=mavutil.mavlink_connection('/dev/serial0',baud=115200)
master.wait_heartbeat()

def altitude():
    m=master.recv_match(type='GLOBAL_POSITION_INT',blocking=True,timeout=2)
    return None if m is None else m.relative_alt/1000.0

def vz_for_alt(alt):
    if alt is None:return 0
    e=alt-TARGET_ALTITUDE
    if abs(e)<=ALT_DEADBAND:return 0
    return max(-ALT_MAX_VZ,min(ALT_MAX_VZ,e*ALT_KP))

def send_body(vx,vz):
    master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,vx,0,vz,0,0,0,0,0)

pipeline=dai.Pipeline()
left=pipeline.create(dai.node.MonoCamera);right=pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B);right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
stereo=pipeline.create(dai.node.StereoDepth);stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT);left.out.link(stereo.left);right.out.link(stereo.right)
xo=pipeline.create(dai.node.XLinkOut);xo.setStreamName('depth');stereo.depth.link(xo.input)

master.set_mode_apm('GUIDED');master.arducopter_arm();master.motors_armed_wait()
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE)
while True:
    a=altitude()
    if a is not None and a>=ALTITUDE_REACHED:break

history=deque(maxlen=DEPTH_HISTORY)
with dai.Device(pipeline) as dev:
    q=dev.getOutputQueue('depth',4,True);start=time.time()
    while time.time()-start<TEST_SECONDS:
        a=altitude()
        if a is not None and not (MIN_SAFE_ALTITUDE<=a<=MAX_SAFE_ALTITUDE):break
        frame=q.get().getFrame();h,w=frame.shape;roi=frame[int(h*.35):int(h*.70),int(w*.40):int(w*.60)];valid=roi[(roi>=1000)&(roi<=9000)]
        if valid.size:
            history.append(float(np.median(valid))/1000.0)
        distance=float(np.median(history)) if history else None
        vx=0.0
        if distance is not None:
            error=distance-TARGET_DISTANCE_M
            if abs(error)>DISTANCE_DEADBAND_M:
                vx=max(-MAX_FORWARD_SPEED,min(MAX_FORWARD_SPEED,error*DISTANCE_KP))
        vz=vz_for_alt(a)
        print(f'alt={a} distance={distance} vx={vx:+.2f} vz={vz:+.2f}')
        send_body(vx,vz);time.sleep(.2)

send_body(0,0);master.set_mode_apm('LAND')

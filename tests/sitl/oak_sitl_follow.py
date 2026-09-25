import depthai as dai
import blobconverter
import numpy as np
from collections import deque
from pymavlink import mavutil
import math,time

TAKEOFF_ALTITUDE=3.0;TEST_SECONDS=45;TARGET_DISTANCE=3.0;DISTANCE_TOLERANCE=.4;CENTER_TOLERANCE=.10
SEARCH_YAW_RATE_DEG=14.0;TRACK_YAW_RATE_DEG=10.0;FORWARD_SPEED=.45;BACKWARD_SPEED=-.45
PERSON_LABEL=15;CONFIDENCE=.5

master=mavutil.mavlink_connection('udpin:0.0.0.0:14552');master.wait_heartbeat();print('SITL connected')

def send(vx=0,yaw=0):master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,vx,0,0,0,0,0,0,math.radians(yaw))
def alt():
 m=master.recv_match(type='GLOBAL_POSITION_INT',blocking=True,timeout=2);return None if m is None else m.relative_alt/1000
master.set_mode_apm('GUIDED');master.arducopter_arm();master.motors_armed_wait();master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TAKEOFF_ALTITUDE)
while True:
 a=alt()
 if a is not None and a>=TAKEOFF_ALTITUDE*.85:break

p=dai.Pipeline();rgb=p.create(dai.node.ColorCamera);rgb.setPreviewSize(300,300);rgb.setInterleaved(False);left=p.create(dai.node.MonoCamera);right=p.create(dai.node.MonoCamera);left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);left.setBoardSocket(dai.CameraBoardSocket.CAM_B);right.setBoardSocket(dai.CameraBoardSocket.CAM_C);st=p.create(dai.node.StereoDepth);st.setDepthAlign(dai.CameraBoardSocket.CAM_A);left.out.link(st.left);right.out.link(st.right);nn=p.create(dai.node.MobileNetDetectionNetwork);nn.setConfidenceThreshold(CONFIDENCE);nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd',shaves=6));rgb.preview.link(nn.input);xo=p.create(dai.node.XLinkOut);xo.setStreamName('nn');nn.out.link(xo.input);xd=p.create(dai.node.XLinkOut);xd.setStreamName('depth');st.depth.link(xd.input)
xh=deque(maxlen=5);dh=deque(maxlen=7)
try:
 with dai.Device(p) as dev:
  qn=dev.getOutputQueue('nn',4,True);qd=dev.getOutputQueue('depth',4,True);start=time.time()
  while time.time()-start<TEST_SECONDS:
   dets=qn.get().detections;depth=qd.get().getFrame();people=[d for d in dets if d.label==PERSON_LABEL]
   if not people:print('NO PERSON | SEARCH');send(0,SEARCH_YAW_RATE_DEG);time.sleep(.1);continue
   d=max(people,key=lambda z:z.confidence);x=((d.xmin+d.xmax)/2)-.5;xh.append(x);x=float(np.median(xh));h,w=depth.shape;cx=int(((d.xmin+d.xmax)/2)*w);cy=int(((d.ymin+d.ymax)/2)*h);roi=depth[max(0,cy-10):cy+10,max(0,cx-10):cx+10];v=roi[(roi>=400)&(roi<=8000)]
   if v.size:dh.append(float(np.median(v))/1000)
   dist=float(np.median(dh)) if dh else None;yaw=TRACK_YAW_RATE_DEG if x>CENTER_TOLERANCE else (-TRACK_YAW_RATE_DEG if x<-CENTER_TOLERANCE else 0);vx=0
   if dist is not None:vx=FORWARD_SPEED if dist>TARGET_DISTANCE+DISTANCE_TOLERANCE else (BACKWARD_SPEED if dist<TARGET_DISTANCE-DISTANCE_TOLERANCE else 0)
   print(f'TRACK | x={x:+.2f} d={dist} vx={vx:+.2f} yaw={yaw:+.1f}');send(vx,yaw);time.sleep(.1)
finally:
 send(0,0);master.set_mode_apm('LAND');time.sleep(10);master.arducopter_disarm()

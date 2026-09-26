# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and test purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

import depthai as dai
import blobconverter
import numpy as np
from collections import deque
from pymavlink import mavutil
import time

FC_PORT='COM8';FC_BAUD=115200
RC6_MIDDLE_MIN=1300;RC6_DOWN_MAX=1200;STABLE_READS=5
MISSION_TIME=30
NEUTRAL=1500;THROTTLE_TEST=1250
YAW_LEFT=1400;YAW_RIGHT=1600;PITCH_FORWARD=1400;PITCH_BACK=1600
COMMAND_SECONDS=0.25;NEUTRAL_SECONDS=0.20
CENTER_TOLERANCE=.10;TARGET_DISTANCE=3.0;DISTANCE_TOLERANCE=.4
PERSON_LABEL=15;CONFIDENCE=.5

master=mavutil.mavlink_connection(FC_PORT,baud=FC_BAUD);master.wait_heartbeat();print('Connected to real flight controller');print('PROPS OFF ONLY')

def rc6():
    m=master.recv_match(type='RC_CHANNELS',blocking=True,timeout=2);return None if m is None else m.chan6_raw

def wait_trigger():
    while True:
        v=rc6();print('RC6:',v)
        if v is not None and v>RC6_MIDDLE_MIN:break
    stable=0
    while stable<STABLE_READS:
        v=rc6();print('RC6:',v);stable=stable+1 if v is not None and v<RC6_DOWN_MAX else 0

def override(roll=NEUTRAL,pitch=NEUTRAL,throttle=NEUTRAL,yaw=NEUTRAL):
    master.mav.rc_channels_override_send(master.target_system,master.target_component,roll,pitch,throttle,yaw,65535,65535,65535,65535)

def neutral_override(seconds=NEUTRAL_SECONDS):
    end=time.time()+seconds
    while time.time()<end:override();time.sleep(.05)

def pulse(**kwargs):
    end=time.time()+COMMAND_SECONDS
    while time.time()<end:override(**kwargs);time.sleep(.05)
    neutral_override()

pipeline=dai.Pipeline();rgb=pipeline.create(dai.node.ColorCamera);rgb.setPreviewSize(300,300);rgb.setInterleaved(False);rgb.setFps(30);left=pipeline.create(dai.node.MonoCamera);right=pipeline.create(dai.node.MonoCamera);left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);left.setBoardSocket(dai.CameraBoardSocket.CAM_B);right.setBoardSocket(dai.CameraBoardSocket.CAM_C);st=pipeline.create(dai.node.StereoDepth);st.setDepthAlign(dai.CameraBoardSocket.CAM_A);st.setSubpixel(True);left.out.link(st.left);right.out.link(st.right);nn=pipeline.create(dai.node.MobileNetDetectionNetwork);nn.setConfidenceThreshold(CONFIDENCE);nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd',shaves=6));rgb.preview.link(nn.input);xo=pipeline.create(dai.node.XLinkOut);xo.setStreamName('nn');nn.out.link(xo.input);xd=pipeline.create(dai.node.XLinkOut);xd.setStreamName('depth');st.depth.link(xd.input)

wait_trigger();master.set_mode_apm('STABILIZE');time.sleep(1);master.arducopter_arm();master.motors_armed_wait();print('ARMED confirmed');neutral_override()
xh=deque(maxlen=5);dh=deque(maxlen=7)
try:
  with dai.Device(pipeline) as dev:
    qn=dev.getOutputQueue('nn',4,True);qd=dev.getOutputQueue('depth',4,True);start=time.time()
    while time.time()-start<MISSION_TIME:
      dets=qn.get().detections;frame=qd.get().getFrame();people=[d for d in dets if d.label==PERSON_LABEL]
      if not people:
        print('NO PERSON | tiny search yaw');pulse(yaw=YAW_RIGHT);continue
      d=max(people,key=lambda z:z.confidence);x=((d.xmin+d.xmax)/2)-.5;xh.append(x);x=float(np.median(xh));h,w=frame.shape;cx=int(((d.xmin+d.xmax)/2)*w);cy=int(((d.ymin+d.ymax)/2)*h);roi=frame[max(0,cy-10):cy+10,max(0,cx-10):cx+10];v=roi[(roi>=400)&(roi<=8000)];dist=float(np.median(v))/1000 if v.size else None
      if dist is not None:dh.append(dist);dist=float(np.median(dh))
      if x<-CENTER_TOLERANCE: yaw='TURN LEFT';pulse(yaw=YAW_LEFT)
      elif x>CENTER_TOLERANCE:yaw='TURN RIGHT';pulse(yaw=YAW_RIGHT)
      else:yaw='HOLD YAW'
      if dist is not None and dist>TARGET_DISTANCE+DISTANCE_TOLERANCE:move='FORWARD';pulse(pitch=PITCH_FORWARD)
      elif dist is not None and dist<TARGET_DISTANCE-DISTANCE_TOLERANCE:move='BACK';pulse(pitch=PITCH_BACK)
      else:move='HOVER'
      print(f'TRACK | x={x:.2f} | distance={dist} | {yaw} | {move}')
finally:
  neutral_override();master.arducopter_disarm();print('Disarm command sent')

import depthai as dai
import blobconverter
import numpy as np
from collections import deque
from pymavlink import mavutil
import time

FC_PORT='COM8';FC_BAUD=115200
RC6_MIDDLE_MIN=1300;RC6_DOWN_MAX=1200;STABLE_READS=5
CENTER_TOLERANCE=0.10
TARGET_DISTANCE_M=3.0;DISTANCE_TOLERANCE_M=0.4
MIN_DISTANCE_M=0.4;MAX_DISTANCE_M=8.0;MAX_DISTANCE_JUMP_M=1.5
MISSION_TIME=45
PERSON_LABEL=15;CONFIDENCE=0.50

master=mavutil.mavlink_connection(FC_PORT,baud=FC_BAUD);master.wait_heartbeat();print('Connected to real flight controller');print('PROPS OFF');print('DRY RUN ONLY - no arming, no motor movement')

def rc6():
    m=master.recv_match(type='RC_CHANNELS',blocking=True,timeout=2)
    return None if m is None else m.chan6_raw

def wait_switch():
    print('Move RC6 to MIDDLE first...')
    while True:
        v=rc6()
        if v is not None:
            print('RC6:',v)
            if v>RC6_MIDDLE_MIN:break
    print('Now flip RC6 DOWN to start dry-run...');stable=0
    while stable<STABLE_READS:
        v=rc6()
        if v is None:continue
        print('RC6:',v);stable=stable+1 if v<RC6_DOWN_MAX else 0
    print('RC6 DOWN confirmed. Dry-run started.')

pipeline=dai.Pipeline();rgb=pipeline.create(dai.node.ColorCamera);rgb.setPreviewSize(300,300);rgb.setInterleaved(False);rgb.setFps(30);rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
left=pipeline.create(dai.node.MonoCamera);right=pipeline.create(dai.node.MonoCamera);left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P);left.setBoardSocket(dai.CameraBoardSocket.CAM_B);right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
stereo=pipeline.create(dai.node.StereoDepth);stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT);stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A);stereo.setSubpixel(True);left.out.link(stereo.left);right.out.link(stereo.right)
nn=pipeline.create(dai.node.MobileNetDetectionNetwork);nn.setConfidenceThreshold(CONFIDENCE);nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd',shaves=6));rgb.preview.link(nn.input)
xn=pipeline.create(dai.node.XLinkOut);xn.setStreamName('nn');nn.out.link(xn.input);xd=pipeline.create(dai.node.XLinkOut);xd.setStreamName('depth');stereo.depth.link(xd.input)

xhist=deque(maxlen=5);dhist=deque(maxlen=7);last_good=None

def depth_for(det,frame):
    global last_good
    h,w=frame.shape;cx=int(((det.xmin+det.xmax)/2)*w);cy=int(((det.ymin+det.ymax)/2)*h);roi=frame[max(0,cy-12):min(h,cy+12),max(0,cx-12):min(w,cx+12)];v=roi[(roi>=MIN_DISTANCE_M*1000)&(roi<=MAX_DISTANCE_M*1000)]
    if not v.size:return last_good
    d=float(np.median(v))/1000
    if last_good is not None and abs(d-last_good)>MAX_DISTANCE_JUMP_M:return last_good
    dhist.append(d);last_good=float(np.median(dhist));return last_good

wait_switch()
with dai.Device(pipeline) as dev:
    qn=dev.getOutputQueue('nn',4,True);qd=dev.getOutputQueue('depth',4,True);start=time.time()
    while time.time()-start<MISSION_TIME:
        # MIDDLE or UP stops the dry run immediately.
        m=master.recv_match(type='RC_CHANNELS',blocking=False)
        if m and m.chan6_raw>=RC6_MIDDLE_MIN:
            print('RC6 stop/recovery position detected — ending dry run');break
        dets=qn.get().detections;frame=qd.get().getFrame();people=[d for d in dets if d.label==PERSON_LABEL]
        if not people:
            print('NO PERSON | WOULD SEARCH / SLOW YAW');continue
        d=max(people,key=lambda x:x.confidence);x=((d.xmin+d.xmax)/2)-.5;xhist.append(x);x=float(np.median(xhist));dist=depth_for(d,frame)
        yaw='TURN LEFT' if x<-CENTER_TOLERANCE else ('TURN RIGHT' if x>CENTER_TOLERANCE else 'HOLD YAW')
        if dist is None:move='HOLD - DISTANCE UNKNOWN'
        elif dist>TARGET_DISTANCE_M+DISTANCE_TOLERANCE_M:move='MOVE FORWARD'
        elif dist<TARGET_DISTANCE_M-DISTANCE_TOLERANCE_M:move='BACK UP'
        else:move='HOVER - GOOD DISTANCE'
        dt='unknown' if dist is None else f'{dist:.2f}m'
        print(f'TRACK | x={x:.2f} | distance={dt} | {yaw} | {move}')

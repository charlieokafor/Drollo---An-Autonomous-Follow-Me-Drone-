# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and test purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

import depthai as dai
from pymavlink import mavutil
import time,math

TARGET_ALTITUDE=1.0;ALTITUDE_REACHED=0.85;TRACK_SECONDS=60
ALT_DEADBAND=.08;ALT_KP=.22;ALT_MAX_VZ_NORMAL=.10
HIGH_ALTITUDE_WARN=1.25;HIGH_ALTITUDE_LAND=1.45;LOW_ALTITUDE_LAND=.55;EMERGENCY_DESCENT_VZ=.25
TAKEOFF_CLIMB_VZ=-0.45  # revisions also tested -0.25 and -0.60
TAKEOFF_TIMEOUT=15
DEFAULT_SEARCH_YAW_RATE_DEG=12.0;LOST_TARGET_YAW_RATE_DEG=12.0;YAW_SIGN=1
CENTER_X_TARGET=.50;CENTER_TOLERANCE=.03;YAW_CENTER_KP=30.0;YAW_CENTER_MAX_DEG=10.0;YAW_CENTER_MIN_DEG=2.5
LEFT_EDGE_LIMIT=.25;RIGHT_EDGE_LIMIT=.75;CONFIDENCE_THRESHOLD=.60
BATTERY_FAILSAFE_KEYWORDS=['Battery Failsafe','Battery 1 is low','Battery 2 is low','Battery critical','battery failsafe','battery low']

master=mavutil.mavlink_connection('/dev/serial0',baud=115200);master.wait_heartbeat();latest_alt=None;battery_failsafe_seen=False

def request(mid,us):master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,mid,us,0,0,0,0,0)
request(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,200000);request(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,500000)

def update():
 global latest_alt,battery_failsafe_seen
 while True:
  m=master.recv_match(blocking=False)
  if m is None:break
  if m.get_type()=='DISTANCE_SENSOR':latest_alt=m.current_distance/100
  elif m.get_type()=='STATUSTEXT':
   print('FC MESSAGE:',m.text)
   if any(k.lower() in m.text.lower() for k in BATTERY_FAILSAFE_KEYWORDS):battery_failsafe_seen=True

def set_mode(mode):master.set_mode_apm(mode);print('Requested mode:',mode);time.sleep(2)
def send_velocity(vx,vy,vz,yaw_deg):master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,vx,vy,vz,0,0,0,0,math.radians(yaw_deg))
def land(reason):print('LAND NOW:',reason);send_velocity(0,0,0,0);set_mode('LAND')
def alt_vz(a):
 if a is None:return 0
 if a>=HIGH_ALTITUDE_WARN:return EMERGENCY_DESCENT_VZ
 e=a-TARGET_ALTITUDE
 return 0 if abs(e)<ALT_DEADBAND else max(-ALT_MAX_VZ_NORMAL,min(ALT_MAX_VZ_NORMAL,e*ALT_KP))
def yaw_center(cx):
 o=cx-CENTER_X_TARGET
 if abs(o)<=CENTER_TOLERANCE:return 0
 r=max(YAW_CENTER_MIN_DEG,min(YAW_CENTER_MAX_DEG,abs(o)*YAW_CENTER_KP));return YAW_SIGN*(r if o>0 else -r)
def search(side):return YAW_SIGN*(-LOST_TARGET_YAW_RATE_DEG if side==-1 else (LOST_TARGET_YAW_RATE_DEG if side==1 else DEFAULT_SEARCH_YAW_RATE_DEG))

pipeline=dai.Pipeline();cam=pipeline.create(dai.node.Camera).build();det=pipeline.create(dai.node.DetectionNetwork).build(cam,dai.NNModelDescription('yolov6-nano'));det.setConfidenceThreshold(CONFIDENCE_THRESHOLD);labels=det.getClasses();q=det.out.createOutputQueue();pipeline.start()

# Key June-13 replacement: no fake GPS origin and no MAV_CMD_NAV_TAKEOFF.
set_mode('GUIDED_NOGPS');master.arducopter_arm();master.motors_armed_wait();print('Armed')
start=time.time()
while time.time()-start<TAKEOFF_TIMEOUT:
 update()
 if battery_failsafe_seen:land('battery during takeoff');raise SystemExit
 if latest_alt is not None:print(f'LiDAR altitude: {latest_alt:.2f}m')
 if latest_alt is not None and latest_alt>=ALTITUDE_REACHED:break
 if latest_alt is not None and latest_alt>=HIGH_ALTITUDE_LAND:land('altitude high');raise SystemExit
 send_velocity(0,0,TAKEOFF_CLIMB_VZ,0);time.sleep(.1)
else:
 land('takeoff timeout');raise SystemExit

last_side=0;start=time.time()
with pipeline:
 while pipeline.isRunning() and time.time()-start<TRACK_SECONDS:
  update();a=latest_alt;vz=alt_vz(a)
  if battery_failsafe_seen:land('battery');break
  if a is not None and (a>=HIGH_ALTITUDE_LAND or a<=LOW_ALTITUDE_LAND):land('altitude safety');break
  pkt=q.get();people=[d for d in pkt.detections if labels[d.label]=='person' and d.confidence>=CONFIDENCE_THRESHOLD]
  if not people:yaw=search(last_side);print(f'NO PERSON | alt={a} yaw={yaw:+.1f}');send_velocity(0,0,vz,yaw)
  else:
   p=max(people,key=lambda d:d.confidence);cx=(p.xmin+p.xmax)/2;o=cx-CENTER_X_TARGET
   if cx<=LEFT_EDGE_LIMIT:last_side=-1
   elif cx>=RIGHT_EDGE_LIMIT:last_side=1
   elif o<-CENTER_TOLERANCE:last_side=-1
   elif o>CENTER_TOLERANCE:last_side=1
   yaw=yaw_center(cx);print(f'PERSON | center={cx:.3f} yaw={yaw:+.1f} alt={a}');send_velocity(0,0,vz,yaw)
  time.sleep(.15)

send_velocity(0,0,0,0);set_mode('LAND')

# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and test purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

# This next-stage safety test was designed in the project after the props-off
# motor-response test. It is included because it documents the intended kill/recovery contract.

from pymavlink import mavutil
import time

PORT='COM8';BAUD=115200
RC6_START=999;RC6_LAND=1503;RC6_MANUAL=2000;TOL=120
MAX_AUTONOMY_SECONDS=30
PERSON_LOST_TIMEOUT=1.0

master=mavutil.mavlink_connection(PORT,baud=BAUD);master.wait_heartbeat()

def near(v,t):return abs(v-t)<=TOL

def rc6_now(timeout=.2):
    m=master.recv_match(type='RC_CHANNELS',blocking=True,timeout=timeout);return None if m is None else m.chan6_raw

def stop_and_disarm(reason):
    print('SAFETY STOP:',reason);master.mav.rc_channels_override_send(master.target_system,master.target_component,1500,1500,1500,1500,65535,65535,65535,65535);master.arducopter_disarm()

print('Waiting for RC6 DOWN autonomy enable...')
while True:
    v=rc6_now(1)
    if v is not None and near(v,RC6_START):break
print('Autonomy enabled')

start=time.time();last_person_seen=time.time()
try:
    while time.time()-start<MAX_AUTONOMY_SECONDS:
        v=rc6_now()
        if v is not None and (near(v,RC6_LAND) or near(v,RC6_MANUAL)):
            stop_and_disarm('RC6 manual/land position');break
        # Vision loop would update this timestamp when a person is confirmed.
        person_seen=False
        if person_seen:last_person_seen=time.time()
        if time.time()-last_person_seen>PERSON_LOST_TIMEOUT:
            print('Person lost — stop movement rather than search forever')
            master.mav.rc_channels_override_send(master.target_system,master.target_component,1500,1500,1500,1500,65535,65535,65535,65535)
        time.sleep(.05)
    else:stop_and_disarm('autonomy timeout')
except KeyboardInterrupt:
    stop_and_disarm('keyboard emergency stop')

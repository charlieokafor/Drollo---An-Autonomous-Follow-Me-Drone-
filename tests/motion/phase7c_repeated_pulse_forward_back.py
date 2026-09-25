from pymavlink import mavutil
import time

MAX_FORWARD_SPEED=0.05
PULSE_SECONDS=0.25
SETTLE_SECONDS=1.0
MOVE_ALT_MIN=0.90
MOVE_ALT_MAX=1.15
PULSES=[+1,+1,-1,-1]

master=mavutil.mavlink_connection('/dev/serial0',baud=115200);master.wait_heartbeat()

def altitude():
    m=master.recv_match(type='GLOBAL_POSITION_INT',blocking=True,timeout=2)
    return None if m is None else m.relative_alt/1000

def send_vx(vx):
    master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,vx,0,0,0,0,0,0,0)

for i,direction in enumerate(PULSES,1):
    a=altitude()
    if a is None or not (MOVE_ALT_MIN<=a<=MOVE_ALT_MAX):
        print(f'Abort pulse {i}: altitude={a}');break
    vx=direction*MAX_FORWARD_SPEED
    print(f'Pulse {i}: vx={vx:+.2f}')
    end=time.time()+PULSE_SECONDS
    while time.time()<end:
        send_vx(vx);time.sleep(.05)
    send_vx(0);time.sleep(SETTLE_SECONDS)
send_vx(0)

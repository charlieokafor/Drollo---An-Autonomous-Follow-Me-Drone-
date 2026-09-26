# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants, outputs and test purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

from pymavlink import mavutil
import time

PORT='/dev/serial0';BAUD=115200
master=mavutil.mavlink_connection(PORT,baud=BAUD)
print('Waiting for heartbeat...');master.wait_heartbeat();print('Connected to FC');print(f'System: {master.target_system}, Component: {master.target_component}');print('PROPS OFF ONLY')

def armed_from(hb):return bool(hb.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def listen(seconds=5):
    print(f'Listening for current FC messages for {seconds} seconds...');end=time.time()+seconds
    while time.time()<end:
        m=master.recv_match(blocking=True,timeout=.5)
        if not m:continue
        t=m.get_type()
        if t=='HEARTBEAT':print(f'HEARTBEAT: mode={master.flightmode} armed={armed_from(m)}')
        elif t=='SYS_STATUS':
            v=m.voltage_battery/1000 if m.voltage_battery!=65535 else None;c=m.current_battery/100 if m.current_battery!=-1 else None;print(f'BATTERY: voltage={v}V current={c}')
        elif t=='RC_CHANNELS':print(f'RC: ch1={m.chan1_raw} ch2={m.chan2_raw} ch3={m.chan3_raw} ch4={m.chan4_raw} ch5={m.chan5_raw} ch6={m.chan6_raw}')
        elif t=='GPS_RAW_INT':print(f'GPS: fix={m.fix_type} sats={m.satellites_visible} hdop={None if m.eph==65535 else m.eph/100}')
        elif t=='EKF_STATUS_REPORT':print(f'EKF: flags={m.flags} vel_var={m.velocity_variance:.3f} pos_var={m.pos_horiz_variance:.3f} compass_var={m.compass_variance:.3f}')
        elif t=='STATUSTEXT':print('FC MESSAGE:',m.text)
        elif t=='COMMAND_ACK':print(f'COMMAND_ACK: command={m.command} result={m.result}')

listen(5)
print('Requesting GUIDED...');master.set_mode_apm('GUIDED');listen(2)
print('Sending ARM...');master.arducopter_arm();listen(8)
hb=master.recv_match(type='HEARTBEAT',blocking=True,timeout=2);state=False if hb is None else armed_from(hb);print('Final armed state:',state);print('Debug complete.');print('Result:', 'ARMED' if state else 'FC refused to arm.');print('Sending DISARM for safety...');master.arducopter_disarm();print('Done.')

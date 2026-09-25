from pymavlink import mavutil
import math,time

TARGET_ALTITUDE=1.0;ALTITUDE_REACHED=.85;YAW_RATE_DEG=12.0;YAW_SECONDS=5.0
master=mavutil.mavlink_connection('/dev/serial0',baud=115200);master.wait_heartbeat()

def yaw(rate):master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,0,0,0,0,0,0,0,math.radians(rate))
master.set_mode_apm('GUIDED');time.sleep(1);master.arducopter_arm();master.motors_armed_wait();master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE)
while True:
 m=master.recv_match(type='GLOBAL_POSITION_INT',blocking=True,timeout=2)
 if m and m.relative_alt/1000>=ALTITUDE_REACHED:break
print('Takeoff complete — yaw-only segment');end=time.time()+YAW_SECONDS
while time.time()<end:yaw(YAW_RATE_DEG);time.sleep(.1)
yaw(0);time.sleep(1);master.set_mode_apm('LAND');print('LAND')

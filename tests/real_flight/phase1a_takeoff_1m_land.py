# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Where a standalone source filename was not retained, this file isolates the
# exact staged behavior that was previously tested inside a larger phase script.
# EXPERIMENTAL UAV SOFTWARE — review configuration and safety before use.

from pymavlink import mavutil
import time

TARGET_ALTITUDE=1.0;ALTITUDE_REACHED=.85;HOVER_SECONDS=10
master=mavutil.mavlink_connection('/dev/serial0',baud=115200);master.wait_heartbeat()
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,200000,0,0,0,0,0)
master.set_mode_apm('GUIDED');time.sleep(1);master.arducopter_arm();master.motors_armed_wait()
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE)
while True:
 m=master.recv_match(type='GLOBAL_POSITION_INT',blocking=True,timeout=2)
 if m:
  alt=m.relative_alt/1000;print(f'Altitude: {alt:.2f}m')
  if alt>=ALTITUDE_REACHED:break
print('1m takeoff threshold reached');time.sleep(HOVER_SECONDS);master.set_mode_apm('LAND');print('LAND')

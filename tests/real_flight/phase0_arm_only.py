from pymavlink import mavutil
import time

PORT='/dev/serial0';BAUD=115200;ARM_HOLD_SECONDS=5
master=mavutil.mavlink_connection(PORT,baud=BAUD);master.wait_heartbeat();print('Connected');print('PROPS OFF ONLY')
master.set_mode_apm('STABILIZE');time.sleep(1);master.arducopter_arm()
try:master.motors_armed_wait(timeout=10)
except Exception:raise RuntimeError('Arm did not confirm')
print('ARMED');time.sleep(ARM_HOLD_SECONDS);master.arducopter_disarm();master.motors_disarmed_wait();print('DISARMED')

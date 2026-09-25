# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify port, mode, parameters and safety before use.

from pymavlink import mavutil
import time

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
MAX_HOVER_SECONDS = 20
CH6_LAND = 1503
CH6_TOLERANCE = 120

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
master.set_mode_apm("GUIDED")
master.arducopter_arm()
master.motors_armed_wait()

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE,
)

while True:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg and msg.relative_alt / 1000.0 >= ALTITUDE_REACHED:
        break

print("At altitude. CH6 middle requests LAND.")
start = time.time()
while time.time() - start < MAX_HOVER_SECONDS:
    msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
    if msg:
        ch6 = msg.chan6_raw
        print("CH6:", ch6)
        if abs(ch6 - CH6_LAND) <= CH6_TOLERANCE:
            master.set_mode_apm("LAND")
            print("CH6 LAND override accepted")
            break
else:
    print("Hover window ended without CH6 LAND request")

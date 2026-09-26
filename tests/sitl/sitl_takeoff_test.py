# RESTORED PROJECT TEST ARTIFACT
# This file was restored from the project's original ChatGPT code history.
# Constants, filenames, phase purpose and control structure are preserved wherever
# the history retained them. Where the original source bytes were unavailable,
# glue/boilerplate was reconstructed to make the historical test a standalone file.
# EXPERIMENTAL UAV SOFTWARE — review configuration and test safely.

from pymavlink import mavutil
import time

TAKEOFF_ALTITUDE = 3.0

# Original SITL endpoint used for the direct ArduPilot TCP test.
master = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected: system={master.target_system} component={master.target_component}")

master.set_mode_apm("GUIDED")
time.sleep(1)
master.arducopter_arm()
master.motors_armed_wait()
print("ARMED")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    TAKEOFF_ALTITUDE,
)
print(f"Takeoff command sent: {TAKEOFF_ALTITUDE:.1f} m")

# The historical test intentionally did not LAND. It hovered until Ctrl+C.
try:
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg:
            altitude = msg.relative_alt / 1000.0
            print(f"Altitude: {altitude:.2f} m")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Interrupted; SITL remains under ArduPilot control.")

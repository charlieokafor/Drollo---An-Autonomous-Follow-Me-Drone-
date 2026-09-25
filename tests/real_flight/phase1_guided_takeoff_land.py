# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify port, mode, parameters and safety before use.

from pymavlink import mavutil
import time

TAKEOFF_ALTITUDE = 3.0
HOVER_SECONDS = 10

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print("Requested mode:", mode)

def wait_for_mode(mode, timeout=10):
    start = time.time()
    while time.time() - start < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and master.flightmode == mode:
            return True
    return False

def wait_until_armed(timeout=15):
    start = time.time()
    while time.time() - start < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print("Armed:", armed)
            if armed:
                return True
    return False

set_mode("GUIDED")
wait_for_mode("GUIDED")
master.arducopter_arm()
if not wait_until_armed():
    raise RuntimeError("Failed to arm")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TAKEOFF_ALTITUDE,
)
print(f"Takeoff command sent: {TAKEOFF_ALTITUDE} m")
time.sleep(HOVER_SECONDS)
set_mode("LAND")
print("LAND command sent")

from pymavlink import mavutil
import time

# A standalone 2 m version restored from the project's Guided-GPS 2 m
# takeoff test discussion. The original separate filename was not retained.
TARGET_ALTITUDE = 2.0
ALTITUDE_REACHED = 1.70  # same 85% reached rule used by the staged tests
HOVER_SECONDS = 10

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
master.set_mode_apm("GUIDED")
time.sleep(1)
master.arducopter_arm()
master.motors_armed_wait()

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE,
)

while True:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg:
        alt = msg.relative_alt / 1000.0
        print(f"Altitude: {alt:.2f}m")
        if alt >= ALTITUDE_REACHED:
            break

time.sleep(HOVER_SECONDS)
master.set_mode_apm("LAND")
print("LAND")

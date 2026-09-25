# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and phase purpose are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify configuration and test safely.

from pymavlink import mavutil
import math
import time

# Extracted as a standalone file from the project's yaw-speed tuning work.
# 3 deg/s was the first fixed value; 12 deg/s became the successful search rate.
YAW_RATES_DEG_S = [3.0, 4.0, 6.0, 8.0, 12.0]
SECONDS_PER_RATE = 2.5

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")

def send_yaw_rate(rate_deg_s):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        1479,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, math.radians(rate_deg_s),
    )

print("This test assumes the aircraft is already safely airborne in GUIDED.")
for rate in YAW_RATES_DEG_S:
    print(f"RIGHT: {rate:.1f} deg/s")
    end = time.time() + SECONDS_PER_RATE
    while time.time() < end:
        send_yaw_rate(rate)
        time.sleep(0.1)
    send_yaw_rate(0)
    time.sleep(1)

    print(f"LEFT: {rate:.1f} deg/s")
    end = time.time() + SECONDS_PER_RATE
    while time.time() < end:
        send_yaw_rate(-rate)
        time.sleep(0.1)
    send_yaw_rate(0)
    time.sleep(1)

print("Yaw sweep complete")

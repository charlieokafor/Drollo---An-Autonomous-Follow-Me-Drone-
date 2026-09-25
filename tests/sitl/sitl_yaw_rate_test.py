from pymavlink import mavutil
import math
import time
# inside the staged scan tests. Rates reflect the historical 3 -> 12 deg/s work.
RATES_DEG_S = [3.0, 6.0, 12.0]
RATE_DURATION = 3.0

master = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
master.wait_heartbeat()
master.set_mode_apm("GUIDED")

def yaw_rate(rate_deg_s, duration):
    end = time.time() + duration
    while time.time() < end:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            1479,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, math.radians(rate_deg_s),
        )
        time.sleep(0.1)

for rate in RATES_DEG_S:
    print(f"Yaw test: +{rate:.1f} deg/s")
    yaw_rate(rate, RATE_DURATION)
    yaw_rate(0, 1.0)
    print(f"Yaw test: -{rate:.1f} deg/s")
    yaw_rate(-rate, RATE_DURATION)
    yaw_rate(0, 1.0)

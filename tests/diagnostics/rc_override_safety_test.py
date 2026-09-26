# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify port, mode, parameters and safety before use.

from pymavlink import mavutil
import time

PORT = "/dev/serial0"
BAUD = 115200
CH6_AUTONOMY = 999
CH6_LAND = 1503
CH6_MANUAL = 2000
TOLERANCE = 120

master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()

def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )

def near(value, target):
    return abs(value - target) <= TOLERANCE

def set_mode(mode):
    master.set_mode_apm(mode)
    print(f"[COMMAND] Switch mode to {mode}")

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 200000)  # 5 Hz
last_mode = master.flightmode

while True:
    msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
    if not msg:
        continue
    ch6 = msg.chan6_raw
    if near(ch6, CH6_MANUAL):
        print(f"CH6={ch6} | RC OVERRIDE → switch to STABILIZE")
        set_mode("STABILIZE")
        last_mode = "STABILIZE"
    elif near(ch6, CH6_LAND):
        print(f"CH6={ch6} | LAND REQUEST → later this will LAND")
    elif near(ch6, CH6_AUTONOMY):
        print(f"CH6={ch6} | AUTONOMY START → later this will GUIDED/FOLLOW")
    else:
        print(f"CH6={ch6} | mode={last_mode}")
    time.sleep(0.05)

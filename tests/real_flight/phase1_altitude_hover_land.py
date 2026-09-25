from pymavlink import mavutil
import time

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
HOVER_SECONDS = 10

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")

def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )

def set_mode(mode):
    master.set_mode_apm(mode)
    print("Mode requested:", mode)
    time.sleep(1)

def wait_until_armed(timeout=15):
    start = time.time()
    while time.time() - start < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False

def altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000)
set_mode("GUIDED")
master.arducopter_arm()
if not wait_until_armed():
    raise RuntimeError("Arm failed")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE,
)

while True:
    alt = altitude()
    if alt is not None:
        print(f"Altitude: {alt:.2f}m")
        if alt >= ALTITUDE_REACHED:
            break
    time.sleep(0.1)

print(f"Altitude reached. Hovering {HOVER_SECONDS}s...")
end = time.time() + HOVER_SECONDS
while time.time() < end:
    alt = altitude()
    if alt is not None:
        print(f"Hover altitude: {alt:.2f}m")

print("Hover test complete. Use CH6 LAND switch to land manually.")

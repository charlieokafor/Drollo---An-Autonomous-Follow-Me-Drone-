from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

def is_armed():
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if msg is None:
        return False
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

print("Sending arm command...")
master.arducopter_arm()
for _ in range(10):
    if is_armed():
        print("ARMED")
        break
    print("Waiting for arm...")
    time.sleep(1)
else:
    raise RuntimeError("Vehicle did not arm")

print("Holding armed state for 5 seconds...")
time.sleep(5)

print("Sending disarm command...")
master.arducopter_disarm()
for _ in range(10):
    if not is_armed():
        print("DISARMED")
        break
    print("Waiting for disarm...")
    time.sleep(1)

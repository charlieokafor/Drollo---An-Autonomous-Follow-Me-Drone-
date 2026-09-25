# RESTORED PROJECT TEST ARTIFACT
# Restored from the project's recorded code and conversation history.
# Historical constants and test intent are preserved where available.
# EXPERIMENTAL UAV SOFTWARE — verify port, mode, parameters and safety before use.

from pymavlink import mavutil
import time

FC_PORT = "COM8"  # COM5 was used first; later bench runs used COM8.
FC_BAUD = 115200
RC6_MIDDLE_MIN = 1300
RC6_DOWN_MAX = 1200
STABLE_READS = 5
ARM_HOLD_SECONDS = 5

master = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
print("Waiting for heartbeat...")
master.wait_heartbeat(timeout=10)
print("Connected")

def rc6_value():
    msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
    return None if msg is None else msg.chan6_raw

def print_status_text(seconds=1.0):
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if msg:
            print("FC MESSAGE:", msg.text)
        time.sleep(0.05)

print("Move RC6 to MIDDLE...")
while True:
    value = rc6_value()
    if value is not None:
        print("RC6:", value)
        if value > RC6_MIDDLE_MIN:
            break

print("Flip RC6 DOWN to arm test...")
stable = 0
while stable < STABLE_READS:
    value = rc6_value()
    if value is None:
        continue
    print("RC6:", value)
    stable = stable + 1 if value < RC6_DOWN_MAX else 0

# GUIDED was tried first. STABILIZE was the successful bench fallback when
# position-estimate checks prevented GUIDED arming.
mode = "GUIDED"
print("Requesting", mode)
master.set_mode_apm(mode)
time.sleep(1)
master.arducopter_arm()
print_status_text(2)

try:
    master.motors_armed_wait(timeout=5)
except Exception:
    print("GUIDED arm did not confirm; trying STABILIZE")
    master.set_mode_apm("STABILIZE")
    time.sleep(1)
    master.arducopter_arm()
    master.motors_armed_wait()

print("ARMED confirmed")
time.sleep(ARM_HOLD_SECONDS)
master.arducopter_disarm()
master.motors_disarmed_wait()
print("DISARMED confirmed")

from pymavlink import mavutil
import time

# RC6 historical values: DOWN=999, MIDDLE=1503, UP=2000.
FC_PORT = "COM5"
FC_BAUD = 115200
MIDDLE_MIN = 1300
DOWN_MAX = 1200
STABLE_READS = 5

master = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
master.wait_heartbeat()
print("Connected")

def rc6():
    msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
    return None if msg is None else msg.chan6_raw

print("Move RC6 to MIDDLE first...")
while True:
    value = rc6()
    if value is None:
        continue
    print("RC6:", value)
    if value > MIDDLE_MIN:
        break

print("Now flip RC6 DOWN...")
stable = 0
while stable < STABLE_READS:
    value = rc6()
    if value is None:
        continue
    print("RC6:", value)
    if value < DOWN_MAX:
        stable += 1
    else:
        stable = 0
    time.sleep(0.05)

print("RC6 DOWN confirmed — trigger condition passed.")

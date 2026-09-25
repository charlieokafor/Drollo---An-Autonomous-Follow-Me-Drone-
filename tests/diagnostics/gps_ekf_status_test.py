from pymavlink import mavutil
import time

PORT = "/dev/serial0"
BAUD = 115200
TEST_SECONDS = 30

master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()
print("Connected")

end = time.time() + TEST_SECONDS
while time.time() < end:
    msg = master.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    kind = msg.get_type()
    if kind == "GPS_RAW_INT":
        print(
            f"GPS: fix={msg.fix_type} sats={msg.satellites_visible} "
            f"hdop={None if msg.eph == 65535 else msg.eph / 100.0}"
        )
    elif kind == "EKF_STATUS_REPORT":
        print(
            f"EKF: flags={msg.flags} vel_var={msg.velocity_variance:.3f} "
            f"pos_var={msg.pos_horiz_variance:.3f} compass_var={msg.compass_variance:.3f}"
        )
    elif kind == "STATUSTEXT":
        print("FC MESSAGE:", msg.text)

print("Desired GPS condition for GPS-GUIDED tests: fix_type >= 3 and no 'Need Position Estimate'.")

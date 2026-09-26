# RESTORED PROJECT TEST ARTIFACT
# This file was restored from the project's original ChatGPT code history.
# Constants, filenames, phase purpose and control structure are preserved wherever
# the history retained them. Where the original source bytes were unavailable,
# glue/boilerplate was reconstructed to make the historical test a standalone file.
# EXPERIMENTAL UAV SOFTWARE — review configuration and test safely.

from pymavlink import mavutil
import time

# Standalone extraction of the forward/backward BODY_NED behavior used in the
# original SITL follow tests.
FORWARD_SPEED = 0.5
BACKWARD_SPEED = -0.5
MOVE_SECONDS = 3.0

master = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
master.wait_heartbeat()
master.set_mode_apm("GUIDED")

def send_vx(vx, duration):
    end = time.time() + duration
    while time.time() < end:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            1479,
            0, 0, 0,
            vx, 0, 0,
            0, 0, 0,
            0, 0,
        )
        time.sleep(0.1)

print("Forward")
send_vx(FORWARD_SPEED, MOVE_SECONDS)
print("Stop")
send_vx(0.0, 1.0)
print("Backward")
send_vx(BACKWARD_SPEED, MOVE_SECONDS)
print("Stop")
send_vx(0.0, 1.0)

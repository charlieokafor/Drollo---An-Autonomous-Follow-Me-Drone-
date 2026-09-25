from pymavlink import mavutil
import time
import math

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
ALTITUDE_BAND = 0.08
VERTICAL_CORRECTION_SPEED = 0.22
HOLD_SECONDS = 60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")

def set_mode(name):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()[name]
    )
    time.sleep(1)

def send_body_velocity(vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0, 0, 0, 0, 0, vz, 0, 0, 0, 0, 0
    )

def get_altitude():
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)
    return None if msg is None else msg.current_distance / 100.0

set_mode("GUIDED")
master.arducopter_arm()
master.motors_armed_wait(timeout=15)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
)

while True:
    alt = get_altitude()
    if alt is not None:
        print(f"Altitude: {alt:.2f}m")
        if alt >= ALTITUDE_REACHED:
            break

print("Altitude-only hold test: no yaw, no forward/back")
end = time.time() + HOLD_SECONDS
while time.time() < end:
    alt = get_altitude()
    if alt is None:
        send_body_velocity(0)
        continue
    if alt < TARGET_ALTITUDE - ALTITUDE_BAND:
        vz = -VERTICAL_CORRECTION_SPEED
    elif alt > TARGET_ALTITUDE + ALTITUDE_BAND:
        vz = VERTICAL_CORRECTION_SPEED
    else:
        vz = 0
    print(f"alt={alt:.2f}m vz={vz:+.2f}")
    send_body_velocity(vz)
    time.sleep(0.1)

send_body_velocity(0)
set_mode("LAND")

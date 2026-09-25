from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()
print("Connected")
print("PROPS OFF / CONTROLLED TEST AREA ONLY")

def set_mode(name):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()[name]
    )
    time.sleep(1)

def send_thrust(thrust):
    # attitude quaternion [1,0,0,0] = level attitude
    master.mav.set_attitude_target_send(
        0,
        master.target_system,
        master.target_component,
        0b00000111,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, 0.0,
        thrust
    )

set_mode("GUIDED_NOGPS")
master.arducopter_arm()
master.motors_armed_wait(timeout=15)

# Historical diagnostic ramp discussed after the velocity climb stayed at ~0.02m.
for thrust in (0.55, 0.60, 0.65, 0.70, 0.75, 0.80):
    print(f"Testing thrust={thrust:.2f}")
    end = time.time() + 1.0
    while time.time() < end:
        send_thrust(thrust)
        time.sleep(0.05)

set_mode("LAND")

from pymavlink import mavutil
import time
import math

CONNECTION = "/dev/serial0"
BAUD = 115200

master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected: system={master.target_system} component={master.target_component}")

def request_interval(message_id, hz):
    interval_us = int(1_000_000 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0
    )

def set_mode(mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Mode {mode} not available")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    time.sleep(1.0)

def wait_armed(timeout=15):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False

def arm(mode="GUIDED"):
    set_mode(mode)
    master.arducopter_arm()
    if not wait_armed():
        raise RuntimeError("Arm timeout")
    print("Armed")

def land():
    set_mode("LAND")
    print("LAND requested")

def takeoff(altitude_m):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude_m
    )
    print(f"Takeoff requested: {altitude_m:.1f} m")

def relative_altitude_m(timeout=2):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if not msg:
        return None
    return msg.relative_alt / 1000.0

def send_body_velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate_deg=0.0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, math.radians(yaw_rate_deg)
    )

TARGET_ALTITUDE=1.0
FORWARD_SPEED=0.25
BACKWARD_SPEED=-0.25
LEG_SECONDS=3.0
arm("GUIDED");takeoff(TARGET_ALTITUDE);time.sleep(4)
for label,vx in (("FORWARD",FORWARD_SPEED),("STOP",0.0),("BACKWARD",BACKWARD_SPEED),("STOP",0.0)):
    print(label)
    end=time.time()+(1.0 if label=="STOP" else LEG_SECONDS)
    while time.time()<end:
        send_body_velocity(vx,0,0,0);time.sleep(.1)
land()

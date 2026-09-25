import depthai as dai
from pymavlink import mavutil
import time
import math

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
YAW_RATE_DEG_S = 12.0
PERSON_LABEL = "person"
CONFIDENCE_THRESHOLD = 0.60

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0
    )

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000)

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1)

def wait_until_armed(timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False

def get_altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def send_velocity_body(vx, vy, vz, yaw_rate_deg):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, math.radians(yaw_rate_deg)
    )

def stop_motion():
    send_velocity_body(0, 0, 0, 0)

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(
    camera, dai.NNModelDescription("yolov6-nano")
)
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

set_mode("GUIDED")
master.arducopter_arm()
if not wait_until_armed():
    raise RuntimeError("Failed to arm")

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

print("Starting yaw search...")
with pipeline:
    while pipeline.isRunning():
        detections = q_det.get().detections
        person = None
        for det in detections:
            if labels[det.label] == PERSON_LABEL and det.confidence >= CONFIDENCE_THRESHOLD:
                person = det
                break
        if person is not None:
            print("Person detected. Stopping yaw.")
            stop_motion()
            time.sleep(5)
            break
        send_velocity_body(0, 0, 0, YAW_RATE_DEG_S)
        time.sleep(0.1)

set_mode("LAND")

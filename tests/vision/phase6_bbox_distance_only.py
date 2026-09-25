import csv
import depthai as dai
from pymavlink import mavutil
import math
import time

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85
ALTITUDE_BAND = 0.12
VERTICAL_CORRECTION_SPEED = 0.18
MIN_SAFE_ALTITUDE = 0.35
MAX_SAFE_ALTITUDE = 1.80
CONFIDENCE_THRESHOLD = 0.60
TEST_SECONDS = 60
CSV_FILE = "phase6_bbox_distance_log.csv"

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
master.wait_heartbeat()

def altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0

def altitude_vz(alt):
    if alt is None: return 0.0
    if alt > TARGET_ALTITUDE + ALTITUDE_BAND: return VERTICAL_CORRECTION_SPEED
    if alt < TARGET_ALTITUDE - ALTITUDE_BAND: return -VERTICAL_CORRECTION_SPEED
    return 0.0

def hold_altitude(vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0,0,0, 0,0,vz, 0,0,0, 0,0,
    )

pipeline = dai.Pipeline()
camera = pipeline.create(dai.node.Camera).build()
detection = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription("yolov6-nano"))
detection.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels = detection.getClasses()
q_det = detection.out.createOutputQueue()
pipeline.start()

master.set_mode_apm("GUIDED")
master.arducopter_arm(); master.motors_armed_wait()
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0,TARGET_ALTITUDE)
while True:
    alt = altitude()
    if alt is not None and alt >= ALTITUDE_REACHED: break

with open(CSV_FILE, "w", newline="") as f, pipeline:
    writer = csv.writer(f)
    writer.writerow(["time_sec","altitude_m","confidence","center_x","box_width","box_height","box_area"])
    start = time.time()
    while pipeline.isRunning() and time.time() - start < TEST_SECONDS:
        alt = altitude()
        if alt is not None and not (MIN_SAFE_ALTITUDE <= alt <= MAX_SAFE_ALTITUDE):
            print("Altitude safety limit reached")
            break
        hold_altitude(altitude_vz(alt))
        packet = q_det.get()
        people = [d for d in packet.detections if labels[d.label] == "person" and d.confidence >= CONFIDENCE_THRESHOLD]
        if not people: continue
        d = max(people, key=lambda x: x.confidence)
        cx=(d.xmin+d.xmax)/2; bw=d.xmax-d.xmin; bh=d.ymax-d.ymin; area=bw*bh
        t=time.time()-start
        writer.writerow([f"{t:.2f}",alt,f"{d.confidence:.3f}",f"{cx:.4f}",f"{bw:.4f}",f"{bh:.4f}",f"{area:.5f}"])
        print(f"t={t:.1f}s alt={alt:.2f} area={area:.4f} cx={cx:.3f}")

hold_altitude(0)
master.set_mode_apm("LAND")
# Historical observations: area <~0.010 far; 0.011–0.016 useful/good; >~0.018 close.

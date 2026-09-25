import time
import depthai as dai
import blobconverter
from pymavlink import mavutil

FC_PORT = "COM5"
FC_BAUD = 115200
PERSON_LABEL = 15
CONFIDENCE = 0.50
ARM_SECONDS = 5
FORCE_ARM_MAGIC = 21196  # historical force-arm value used during bench testing

master = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
print("Waiting for FC heartbeat...")
master.wait_heartbeat()
print("Connected to flight controller")

def armed_state(timeout=3):
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if not msg:
        return None
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def arm(force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        FORCE_ARM_MAGIC if force else 0,
        0, 0, 0, 0, 0,
    )
    print("ARM command sent")

def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    print("DISARM command sent")

# Early OAK-D MobileNet-SSD pipeline used on the Windows bench.
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(300, 300)
cam.setInterleaved(False)
cam.setFps(30)

nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
n.setConfidenceThreshold(CONFIDENCE)
n.setBlobPath(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6))
cam.preview.link(nn.input)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("detections")
nn.out.link(xout.input)

print("Initial armed state:", armed_state())

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("detections", maxSize=4, blocking=True)
    try:
        while True:
            detections = q.get().detections
            if any(det.label == PERSON_LABEL for det in detections):
                print("Person detected")
                arm(force=True)
                state = armed_state(5)
                print("REAL FC ARMED STATE:", state)
                time.sleep(ARM_SECONDS)
                disarm()
                print("REAL FC ARMED STATE:", armed_state(5))
                break
            print("No person")
    finally:
        disarm()

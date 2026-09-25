import depthai as dai
import cv2
import time
import os

OUTPUT_DIR = "oakd_test_images"
os.makedirs(OUTPUT_DIR, exist_ok=True)

devices = dai.Device.getAllAvailableDevices()
if not devices:
    raise SystemExit("No OAK-D device found")

for dev in devices:
    print(f"Device: name={dev.name} mxid={dev.deviceId} state={dev.state}")

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(640, 480)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("preview")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
    captured = 0
    deadline = time.time() + 20
    while captured < 5 and time.time() < deadline:
        packet = q.tryGet()
        if packet is None:
            time.sleep(0.05)
            continue
        frame = packet.getCvFrame()
        captured += 1
        path = os.path.join(OUTPUT_DIR, f"oakd_capture_{captured}.jpg")
        cv2.imwrite(path, frame)
        print("Saved", path)
        time.sleep(1)

print(f"Captured {captured} frame(s)")

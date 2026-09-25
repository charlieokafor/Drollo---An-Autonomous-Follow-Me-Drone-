import depthai as dai
from pymavlink import mavutil
import time

SEARCH_YAW_RATE=12.0  # BUG: interpreted as rad/s, not 12 deg/s
TARGET_ALTITUDE=1.0
CONFIDENCE_THRESHOLD=0.60

master=mavutil.mavlink_connection("/dev/serial0",baud=115200); master.wait_heartbeat()
mode=master.mode_mapping()["GUIDED"]
master.mav.set_mode_send(master.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode)
master.arducopter_arm(); master.motors_armed_wait(timeout=15)
master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TARGET_ALTITUDE)
time.sleep(4)

pipeline=dai.Pipeline(); camera=pipeline.create(dai.node.Camera).build()
det=pipeline.create(dai.node.DetectionNetwork).build(camera,dai.NNModelDescription("yolov6-nano")); det.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
labels=det.getClasses(); q=det.out.createOutputQueue(); pipeline.start()
with pipeline:
    while pipeline.isRunning():
        packet=q.get(); person=next((d for d in packet.detections if labels[d.label]=="person" and d.confidence>=CONFIDENCE_THRESHOLD),None)
        if person:
            print("Person detected - stop yaw")
            master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,0,0,0,0,0,0,0,0)
            time.sleep(3); break
        master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_BODY_NED,1479,0,0,0,0,0,0,0,0,0,0,SEARCH_YAW_RATE)
        time.sleep(0.15)
mode=master.mode_mapping()["LAND"]; master.mav.set_mode_send(master.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode)

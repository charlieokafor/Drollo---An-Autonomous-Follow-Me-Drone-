import depthai as dai
from pymavlink import mavutil
import time
import math
import statistics
from collections import deque

# ---------------- CONFIG ----------------

TARGET_ALTITUDE = 1.0
ALTITUDE_REACHED = 0.85

# Smooth altitude guard
ALT_DEADBAND = 0.08
ALT_KP = 0.25
ALT_MAX_VZ_NORMAL = 0.10

# Emergency altitude protection
HIGH_ALTITUDE_WARN = 1.20
HIGH_ALTITUDE_LAND = 1.35
EMERGENCY_DESCENT_VZ = 0.30

LOW_ALTITUDE_WARN = 0.80
LOW_ALTITUDE_LAND = 0.60

# Depth control
TARGET_DISTANCE_M = 5.5
DISTANCE_DEADBAND_M = 0.25

# Forward/back control
DISTANCE_KP = 0.17
MAX_FORWARD_SPEED = 0.38

# Longer controlled pulse
PULSE_SECONDS = 3.0
SETTLE_SECONDS = 0.25

TEST_SECONDS = 55

# Forward/back movement only allowed inside this altitude range
MOVE_ALT_MIN = 0.85
MOVE_ALT_MAX = 1.18

# Attitude stability gate
MAX_ROLL_DEG = 7.0
MAX_PITCH_DEG = 7.0
MAX_ROLL_RATE_DEG_S = 45.0
MAX_PITCH_RATE_DEG_S = 45.0
MAX_YAW_RATE_DEG_S = 45.0
STABLE_REQUIRED_COUNT = 2

# Depth filtering
MIN_VALID_DEPTH = 1.0
MAX_VALID_DEPTH = 9.0
DEPTH_SMOOTHING_WINDOW = 5

BATTERY_FAILSAFE_KEYWORDS = [
    "Battery Failsafe",
    "Battery 1 is low",
    "Battery 2 is low",
    "Battery critical",
    "battery failsafe",
    "battery low",
]

# ----------------------------------------

master = mavutil.mavlink_connection("/dev/serial0", baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

latest_alt = None
latest_attitude = None
battery_failsafe_seen = False


def request_message_interval(message_id, interval_us):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


# Altitude at 5 Hz
request_message_interval(
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    200000
)

# Attitude at 10 Hz
request_message_interval(
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    100000
)

# Status text for failsafes
request_message_interval(
    mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
    500000
)


def update_mavlink_cache():
    global latest_alt, latest_attitude, battery_failsafe_seen

    while True:
        msg = master.recv_match(blocking=False)

        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            latest_alt = msg.relative_alt / 1000.0

        elif msg_type == "ATTITUDE":
            latest_attitude = {
                "roll_deg": math.degrees(msg.roll),
                "pitch_deg": math.degrees(msg.pitch),
                "yaw_deg": math.degrees(msg.yaw),
                "rollspeed_deg": math.degrees(msg.rollspeed),
                "pitchspeed_deg": math.degrees(msg.pitchspeed),
                "yawspeed_deg": math.degrees(msg.yawspeed),
            }

        elif msg_type == "STATUSTEXT":
            text = msg.text
            print("FC MESSAGE:", text)

            lower_text = text.lower()

            for key in BATTERY_FAILSAFE_KEYWORDS:
                if key.lower() in lower_text:
                    battery_failsafe_seen = True
                    break


def set_mode(mode):
    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print(f"Requested mode: {mode}")
    time.sleep(1)


def wait_until_armed(timeout=15):
    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None:
            continue

        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True

    return False


def get_altitude_blocking(timeout=2.0):
    start = time.time()

    while time.time() - start < timeout:
        update_mavlink_cache()

        if latest_alt is not None:
            return latest_alt

        time.sleep(0.05)

    return None


def altitude_guard_vz(current_alt):
    if current_alt is None:
        return 0.0

    # LOCAL_NED:
    # +vz = down
    # -vz = up

    if current_alt >= HIGH_ALTITUDE_WARN:
        return EMERGENCY_DESCENT_VZ

    error = current_alt - TARGET_ALTITUDE

    if abs(error) < ALT_DEADBAND:
        return 0.0

    vz = error * ALT_KP

    if vz > ALT_MAX_VZ_NORMAL:
        vz = ALT_MAX_VZ_NORMAL
    elif vz < -ALT_MAX_VZ_NORMAL:
        vz = -ALT_MAX_VZ_NORMAL

    return vz


def is_drone_stable(alt, attitude):
    if alt is None or attitude is None:
        return False, "missing alt/attitude"

    if alt < MOVE_ALT_MIN:
        return False, "alt too low"

    if alt > MOVE_ALT_MAX:
        return False, "alt too high"

    if abs(attitude["roll_deg"]) > MAX_ROLL_DEG:
        return False, "roll too high"

    if abs(attitude["pitch_deg"]) > MAX_PITCH_DEG:
        return False, "pitch too high"

    if abs(attitude["rollspeed_deg"]) > MAX_ROLL_RATE_DEG_S:
        return False, "roll rate too high"

    if abs(attitude["pitchspeed_deg"]) > MAX_PITCH_RATE_DEG_S:
        return False, "pitch rate too high"

    if abs(attitude["yawspeed_deg"]) > MAX_YAW_RATE_DEG_S:
        return False, "yaw rate too high"

    return True, "stable"


def depth_to_forward_speed(depth_m):
    if depth_m is None:
        return 0.0

    error = depth_m - TARGET_DISTANCE_M

    if abs(error) < DISTANCE_DEADBAND_M:
        return 0.0

    vx = error * DISTANCE_KP

    if vx > MAX_FORWARD_SPEED:
        vx = MAX_FORWARD_SPEED
    elif vx < -MAX_FORWARD_SPEED:
        vx = -MAX_FORWARD_SPEED

    return vx


def send_body_velocity(vx, vy, vz, yaw_rate_deg):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        1479,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0,
        math.radians(yaw_rate_deg)
    )


def stop_motion_with_altitude_guard():
    update_mavlink_cache()
    vz = altitude_guard_vz(latest_alt)
    send_body_velocity(0, 0, vz, 0)


def land_now(reason):
    print(f"LAND NOW: {reason}")

    for _ in range(10):
        send_body_velocity(0, 0, 0, 0)
        time.sleep(0.05)

    set_mode("LAND")


def fmt(value, digits=2, suffix=""):
    if value is None:
        return "None"

    return f"{value:.{digits}f}{suffix}"


def median_depth_from_center(depth_frame):
    height, width = depth_frame.shape

    x1 = int(width * 0.40)
    x2 = int(width * 0.60)

    y1 = int(height * 0.35)
    y2 = int(height * 0.70)

    roi = depth_frame[y1:y2, x1:x2]

    values = roi.flatten()
    values = [
        int(v)
        for v in values
        if int(v) > 200 and int(v) < 10000
    ]

    if not values:
        return None

    return statistics.median(values) / 1000.0


# ---------- OAK-D DEPTH ONLY ----------

pipeline = dai.Pipeline()

left = pipeline.create(dai.node.Camera).build(
    dai.CameraBoardSocket.CAM_B
)

right = pipeline.create(dai.node.Camera).build(
    dai.CameraBoardSocket.CAM_C
)

left_out = left.requestOutput(
    (400, 400),
    type=dai.ImgFrame.Type.GRAY8
)

right_out = right.requestOutput(
    (400, 400),
    type=dai.ImgFrame.Type.GRAY8
)

stereo = pipeline.create(dai.node.StereoDepth)

stereo.setDefaultProfilePreset(
    dai.node.StereoDepth.PresetMode.DEFAULT
)

left_out.link(stereo.left)
right_out.link(stereo.right)

q_depth = stereo.depth.createOutputQueue()

pipeline.start()


# ---------- TAKEOFF ----------

print("Switching to GUIDED mode.")
set_mode("GUIDED")

print("Arming.")
master.arducopter_arm()

if not wait_until_armed():
    print("Failed to arm")
    raise SystemExit

print(f"Sending MAV_CMD_NAV_TAKEOFF command to {TARGET_ALTITUDE}m.")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0,
    TARGET_ALTITUDE
)

while True:
    update_mavlink_cache()

    if battery_failsafe_seen:
        land_now("battery failsafe during takeoff")
        raise SystemExit

    alt = get_altitude_blocking()

    if alt is not None:
        print(f"Altitude: {alt:.2f}m")

        if alt >= HIGH_ALTITUDE_LAND:
            land_now(f"altitude too high during takeoff: {alt:.2f}m")
            raise SystemExit

        if alt <= LOW_ALTITUDE_LAND:
            # During initial spool-up this can briefly be near zero,
            # so only treat it as a hard abort once the aircraft has climbed.
            pass

        if alt >= ALTITUDE_REACHED:
            print("Altitude reached.")
            break

    time.sleep(0.2)


# Give altitude / attitude telemetry time to populate
cache_wait_start = time.time()

while time.time() - cache_wait_start < 5.0:
    update_mavlink_cache()

    if latest_alt is not None and latest_attitude is not None:
        break

    time.sleep(0.1)


# ---------- PHASE 7: DEPTH PULSE FOLLOW ----------

print("Starting Phase 7 depth pulse follow test.")
print(f"Target distance: {TARGET_DISTANCE_M:.2f}m")
print(f"Test time: {TEST_SECONDS}s")

depth_history = deque(maxlen=DEPTH_SMOOTHING_WINDOW)

stable_count = 0
mission_should_land = False
land_reason = ""

test_start = time.time()

with pipeline:
    while pipeline.isRunning() and time.time() - test_start < TEST_SECONDS:
        update_mavlink_cache()

        alt = latest_alt
        attitude = latest_attitude

        if battery_failsafe_seen:
            mission_should_land = True
            land_reason = "battery failsafe detected"
            break

        if alt is None:
            print("No altitude data.")
            send_body_velocity(0, 0, 0, 0)
            stable_count = 0
            time.sleep(0.2)
            continue

        if alt >= HIGH_ALTITUDE_LAND:
            mission_should_land = True
            land_reason = f"altitude too high: {alt:.2f}m"
            break

        if alt <= LOW_ALTITUDE_LAND:
            mission_should_land = True
            land_reason = f"altitude too low: {alt:.2f}m"
            break

        if alt >= HIGH_ALTITUDE_WARN:
            print(
                "High altitude warning → stopping horizontal movement, "
                "forcing descent"
            )

            send_body_velocity(
                0,
                0,
                EMERGENCY_DESCENT_VZ,
                0
            )

            stable_count = 0
            time.sleep(0.2)
            continue

        # Hold altitude even when forward/back movement is not allowed
        vz = altitude_guard_vz(alt)

        stable, stable_reason = is_drone_stable(
            alt,
            attitude
        )

        if stable:
            stable_count += 1
        else:
            stable_count = 0

        depth_packet = q_depth.get()
        depth_frame = depth_packet.getFrame()

        raw_depth = median_depth_from_center(
            depth_frame
        )

        if (
            raw_depth is not None
            and MIN_VALID_DEPTH <= raw_depth <= MAX_VALID_DEPTH
        ):
            depth_history.append(raw_depth)

        if len(depth_history) >= 3:
            smoothed_depth = statistics.median(
                depth_history
            )
        else:
            smoothed_depth = None

        vx = depth_to_forward_speed(
            smoothed_depth
        )

        if vx > 0:
            movement = "WOULD MOVE FORWARD"
        elif vx < 0:
            movement = "WOULD MOVE BACKWARD"
        else:
            movement = "HOLD DISTANCE"

        print(
            f"t={time.time() - test_start:.1f}s | "
            f"alt={fmt(alt, 2, 'm')} | "
            f"depth={fmt(smoothed_depth, 2, 'm')} | "
            f"vx={vx:.2f} | "
            f"stable={stable} ({stable_reason}) | "
            f"stable_count={stable_count} | "
            f"{movement}"
        )

        if stable_count < STABLE_REQUIRED_COUNT:
            send_body_velocity(
                0,
                0,
                vz,
                0
            )

            time.sleep(0.2)
            continue

        if vx == 0:
            send_body_velocity(
                0,
                0,
                vz,
                0
            )

            time.sleep(0.2)
            continue

        # ---------- MOVEMENT PULSE ----------

        pulse_start = time.time()

        while time.time() - pulse_start < PULSE_SECONDS:
            update_mavlink_cache()

            alt = latest_alt
            attitude = latest_attitude

            if battery_failsafe_seen:
                mission_should_land = True
                land_reason = "battery failsafe during pulse"
                break

            if alt is None:
                print("Pulse aborted: missing altitude")
                break

            if alt >= HIGH_ALTITUDE_LAND:
                mission_should_land = True
                land_reason = f"altitude too high during pulse: {alt:.2f}m"
                break

            if alt <= LOW_ALTITUDE_LAND:
                mission_should_land = True
                land_reason = f"altitude too low during pulse: {alt:.2f}m"
                break

            if alt >= HIGH_ALTITUDE_WARN:
                print("Pulse aborted: high altitude warning")

                send_body_velocity(
                    0,
                    0,
                    EMERGENCY_DESCENT_VZ,
                    0
                )

                break

            stable, stable_reason = is_drone_stable(
                alt,
                attitude
            )

            if not stable:
                print(
                    f"Pulse aborted: {stable_reason}"
                )
                break

            vz = altitude_guard_vz(alt)

            send_body_velocity(
                vx,
                0,
                vz,
                0
            )

            time.sleep(0.05)

        if mission_should_land:
            break

        # ---------- SETTLE ----------

        settle_start = time.time()

        while time.time() - settle_start < SETTLE_SECONDS:
            update_mavlink_cache()

            alt = latest_alt

            if battery_failsafe_seen:
                mission_should_land = True
                land_reason = "battery failsafe during settle"
                break

            if alt is not None:
                if alt >= HIGH_ALTITUDE_LAND:
                    mission_should_land = True
                    land_reason = (
                        f"altitude too high during settle: "
                        f"{alt:.2f}m"
                    )
                    break

                if alt <= LOW_ALTITUDE_LAND:
                    mission_should_land = True
                    land_reason = (
                        f"altitude too low during settle: "
                        f"{alt:.2f}m"
                    )
                    break

            vz = altitude_guard_vz(alt)

            send_body_velocity(
                0,
                0,
                vz,
                0
            )

            time.sleep(0.05)

        stable_count = 0

        if mission_should_land:
            break

        time.sleep(0.2)


# ---------- LAND ----------

if mission_should_land:
    print(f"Ending test early: {land_reason}")
else:
    print("Phase 7 test complete.")

for _ in range(10):
    stop_motion_with_altitude_guard()
    time.sleep(0.05)

set_mode("LAND")
print("LAND command sent.")

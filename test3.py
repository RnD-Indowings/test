from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavcmd
import time

# =============================
# MAVLink connection
# =============================
mav = mavutil.mavlink_connection(
    'udpin:127.0.0.1:14550',
    source_system=1,
    source_component=mavutil.mavlink.MAV_COMP_ID_CAMERA
)

print("📷 Camera emulator ready (UDP 14550)")

# =============================
# Internal camera state
# =============================
zoom_level = 0          # 0–100 %
focus_value = 0
recording = False

# Gimbal state
pitch = 0.0
roll = 0.0
yaw = 0.0

last_hb = 0

# =============================
# Helper: decode commands
# =============================
def handle_command(msg):
    global zoom_level, focus_value, recording
    global pitch, roll, yaw

    cmd = msg.command

    # 📸 Take photo
    if cmd == mavcmd.MAV_CMD_DO_DIGICAM_CONTROL:
        if msg.param4 == 1:
            print("📸 PHOTO TRIGGERED")

    # 🔍 Zoom
    elif cmd == mavcmd.MAV_CMD_SET_CAMERA_ZOOM:
        zoom_type = int(msg.param1)
        value = msg.param2

        if zoom_type == 0:  # step
            zoom_level += value * 5
        elif zoom_type == 2:  # range
            zoom_level = value

        zoom_level = max(0, min(100, zoom_level))
        print(f"🔍 ZOOM → {zoom_level:.0f}%")

    # 🎯 Focus
    elif cmd == mavcmd.MAV_CMD_SET_CAMERA_FOCUS:
        focus_value = msg.param2
        print(f"🎯 FOCUS → {focus_value}")

    # 🎥 Start video
    elif cmd == mavcmd.MAV_CMD_VIDEO_START_CAPTURE:
        recording = True
        print("🎥 VIDEO RECORDING STARTED")

    # ⏹ Stop video
    elif cmd == mavcmd.MAV_CMD_VIDEO_STOP_CAPTURE:
        recording = False
        print("⏹ VIDEO RECORDING STOPPED")

    # 🎛 Gimbal control (pitch / roll / yaw)
    elif cmd == mavcmd.MAV_CMD_DO_MOUNT_CONTROL:
        pitch = msg.param1 / 100.0
        roll  = msg.param2 / 100.0
        yaw   = msg.param3 / 100.0

        print(
            f"🎛 GIMBAL → "
            f"Pitch={pitch:.2f}° "
            f"Roll={roll:.2f}° "
            f"Yaw={yaw:.2f}°"
        )

    else:
        print(f"❓ UNKNOWN COMMAND: {cmd}")

    # Always ACK
    mav.mav.command_ack_send(
        cmd,
        mavutil.mavlink.MAV_RESULT_ACCEPTED
    )

# =============================
# Main loop
# =============================
while True:
    now = time.time()

    # Send camera heartbeat @ 1 Hz
    if now - last_hb > 1.0:
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_CAMERA,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        last_hb = now

    # Receive messages
    msg = mav.recv_match(blocking=False)
    if not msg:
        continue

    if msg.get_type() == "COMMAND_LONG":
        if msg.target_component in (
            0,
            mavutil.mavlink.MAV_COMP_ID_CAMERA,
            mavutil.mavlink.MAV_COMP_ID_GIMBAL
        ):
            handle_command(msg)

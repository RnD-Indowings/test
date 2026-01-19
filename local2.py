from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection(
    'udpin:127.0.0.1:14550',
    source_system=1,
    source_component=mavutil.mavlink.MAV_COMP_ID_CAMERA
)

print("📷 Camera emulator ready (UDP 14550)")

last_hb = 0

while True:
    now = time.time()

    # Send camera heartbeat
    if now - last_hb > 1.0:
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_CAMERA,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        last_hb = now

    msg = mav.recv_match(blocking=False)
    if not msg:
        continue

    if msg.get_type() == "COMMAND_LONG":
        if msg.target_component in (0, mavutil.mavlink.MAV_COMP_ID_CAMERA):
            print(
                 "CMD:", msg.command,
                 "target_comp:", msg.target_component,
                 "param1:", msg.param1
            )

            mav.mav.command_ack_send(
                msg.command,
                mavutil.mavlink.MAV_RESULT_ACCEPTED
            )

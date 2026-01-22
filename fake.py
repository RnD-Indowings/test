from pymavlink import mavutil
import time

# -------------------------------
# Connect to MAVProxy MAVLink stream
# -------------------------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# -------------------------------
# STEP 1: Configure mount (REQUIRED)
# Equivalent to MAVProxy internal setup
# -------------------------------
print("Configuring mount (MAVLink targeting)...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
    0,
    mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,  # param1: mode
    1,  # param2: stabilize roll
    1,  # param3: stabilize pitch
    1,  # param4: stabilize yaw
    0,  # param5
    0,  # param6
    0   # param7
)

# Wait for ACK of CONFIGURE
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE:
        print("Mount configure ACK:", msg.result)
        break

time.sleep(0.2)

# -------------------------------
# STEP 2: Send gimbal angle command
# Equivalent to: gimbal point 0 45 0
# -------------------------------
pitch = 45   # degrees
roll  = 0
yaw   = 0

print("Sending gimbal pitch command (45 deg)...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
    0,
    pitch,   # param1: pitch (deg)
    roll,    # param2: roll
    yaw,     # param3: yaw
    0,       # param4
    0,       # param5
    0,       # param6
    mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING  # param7
)

# -------------------------------
# STEP 3: Wait for ACCEPT / REJECT
# -------------------------------
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL:
        print("Mount control ACK:", msg.result)

        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("STATUS: ACCEPTED ✅ (Pitch command executed)")
        else:
            print("STATUS: REJECTED ❌")

        break

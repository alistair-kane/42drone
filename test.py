from pymavlink import mavutil
import time
import sys

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# msg = master.recv_match(type='SYS_STATUS', blocking=True)

# GPS_TYPE need to be MAV
while True:
    time.sleep(0.2)
    master.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        0,  # Latitude (WGS84), in degrees * 1E7
        0,  # Longitude (WGS84), in degrees * 1E7
        0,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        7   # Number of satellites visible.
    )

# msg = master.recv_match(type='GPS_RAW_INT')
print(msg)

# master.mav.gps_input_send()

# Arm
# master.arducopter_arm()

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
# print("Waiting for the vehicle to arm")
# master.motors_armed_wait()
# print('Armed!')

# Disarm
# master.arducopter_disarm() or:
# master.mav.command_long_send(
    # master.target_system,
     # master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
# master.motors_disarmed_wait()
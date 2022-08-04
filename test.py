from pymavlink import mavutil
from marvelmind import MarvelMindHedge
import time
import sys

hedge = MarvelMindHedge(tty = "/dev/ttyUSB1", adr=None, debug=False)
hedge.start()

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

while True:
	
master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
# msg = master.recv_match(type='SYS_STATUS', blocking=True)

# msg = master.recv_match(type='GPS_RAW_INT')

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
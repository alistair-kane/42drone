from pymavlink import mavutil
from marvelmind import MarvelMindHedge
import sys

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

def send_gps_data(time, q[4], x, y, z):
	"""
	Updates the drone with Marvelmind external positioning data
	Args:
		time_usec (uint64_t): Timestamp (UNIX Epoch time or time since system boot). 
							The receiving end can infer timestamp format (since 1.1.1970 or since system boot) 
							by checking for the magnitude of the number.
		q[4] (float)		: Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		x (float)			: X position (NED)
		y (float)			: Y position (NED)
		z (float)			: Z position (NED)

	convariance:
	Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle 
	(states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five 
	entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
	"""
	master.mav.att_pos_mocap_send(
		time,
		q,
		x,
		y,
		z,
		NaN)

#can filter the beacon address here?
hedge = MarvelMindHedge(tty = "/dev/ttyUSB1", adr=None, debug=False)
hedge.start()

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

request_message_interval(mavutil.mavlink.AUTOPILOT_VERSION, 10)

while True:
	msg = master.recv_match()
	if not msg:
		continue
	if msg.get_type() == 'HEARTBEAT':
		print("\n\n*****Got message: %s*****" % msg.get_type())
		print("Message: %s" % msg)
		print("\nAs dictionary: %s" % msg.to_dict())
		# Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
		print("\nSystem status: %s" % msg.system_status)




# while True:
# 	try:
# 		hedge.dataEvent.wait(1)
# 		hedge.dataEvent.clear()
# 		if (hedge.fusionImuUpdated)
# 			hedge.print_imu_fusion()
# 	except KeyboardInterrupt:
# 		hedge.stop()
# 		sys.exit()
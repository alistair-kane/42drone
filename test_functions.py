from statistics import covariance
from numpy import NaN, uint64
from pymavlink import mavutil
from marvelmind import MarvelmindHedge
import sys
import time
import numpy as np

def request_message_interval(message_input: str, frequency_hz: float):
	"""
	Request MAVLink message in a desired frequency,
	documentation for SET_MESSAGE_INTERVAL:
		https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

	Args:
		message_id (str): MAVLink message ID
		frequency_hz (float): Desired frequency in Hz
	"""
	message_name = "MAVLINK_MSG_ID_" + message_input
	message_id = getattr(mavutil.mavlink, message_name)
	master.mav.command_long_send(
		master.target_system, master.target_component,
		mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
		message_id,
		1e6 / frequency_hz,
		0,
		0, 0, 0, 0)
	print("Requested the message successfully.")



def get_requested_data(message_name: str, dict_key: str, value_unit: str, save_name: str):
	try:
		message_index = 0
		dict1 = master.recv_match(type= message_name, blocking=True, timeout=0.1).to_dict()
		dict_value = dict1[dict_key]

		toWrite = "Message_Index, " + message_index + " :" + str(dict_value) + value_unit
		with open(save_name, 'a') as file:
			file.write(toWrite)
			file.write('\n')  
			message_index += 1
	except:
		pass


# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_estimate_message(x: float, y: float z: float):
	global current_time_us, H_aeroRef_aeroBody, reset_counter
	with lock:
		covariance = np.zeros(21, dtype=float)
		# Send the message
		master.mav.vision_position_estimate_send(
			current_time_us,            # us Timestamp (UNIX time or time since system boot)
			x,   # Global X position
			y,   # Global Y position
			z,   # Global Z position
			13,	                # Roll angle
			13,	                # Pitch angle
			13,	                # Yaw angle
			covariance,         # Row-major representation of pose 6x6 cross-covariance matrix
			0,               	# Estimate reset counter. Increment every time pose estimate jumps.
		)

def main():

	#can filter the beacon address here?
	hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False)
	hedge.start()

	global master 
	master = mavutil.mavlink_connection("/dev/ttyUSB1", baud=57600)
	master.wait_heartbeat()
	print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

	master = mavutil.mavlink_connection('udpout:localhost:14550', source_system=1)

	request_message_interval("GPS_RAW_INT", 1.0)
	request_message_interval("VFR_HUD", 1.0)

	while True:
		try:
			hedge.dataEvent.wait(1)
			hedge.dataEvent.clear()

			if (hedge.positionUpdated):
				# hedge.print_position()
				data = hedge.position()
				# print(data)
				print("time: %f x: %f y: %f z: %f" % (data[5], data[1], data[2], data[3]))
				# NEED TO CHECK IF THE NED TRANSLATION IS NEEDED HERE
				send_vision_position_estimate_message(data[1], data[2], data[3])
		except KeyboardInterrupt:
			hedge.stop()
			sys.exit()

		# get_requested_data('GPS_RAW_INT', 'fix_type', ' ', 'gps log')
		# get_requested_data('VFR_HUD', 'alt', 'm', 'vfr_log')

		time.sleep(0.1)

main()

"""
setup the router so QGC and the code works
https://github.com/mavlink-router/mavlink-router
	monitor the shit

fix the code based on marvelmind beacons
set the flags on PX4 board - taking vision position

GO FOR LOCK

"""
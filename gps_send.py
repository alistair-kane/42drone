from pymavlink import mavutil
from marvelmind import MarvelmindHedge
import sys
import threading
import queue
import time
import numpy as np
from numpy import NaN


# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
	master.mav.set_gps_global_origin_send(
		1,
		home_lat, 
		home_lon,
		home_alt
	)

def set_default_home_position():
	x = 0
	y = 0
	z = 0
	q = [1, 0, 0, 0]   # w x y z

	approach_x = 0
	approach_y = 0
	approach_z = 1

	master.mav.set_home_position_send(
		1,
		home_lat, 
		home_lon,
		home_alt,
		x,
		y,
		z,
		q,
		approach_x,
		approach_y,
		approach_z
	)

def send_gps_data(cb, xyz):
	"""
	Updates the drone with Marvelmind external positioning data
	Args:
		time_usec (uint64_t): Timestamp (UNIX Epoch time or time since system boot). 
							The receiving end can infer timestamp format (since 1.1.1970 or since system boot) 
							by checking for the magnitude of the number.
		x (float)			: X position (NED)
		y (float)			: Y position (NED)
		z (float)			: Z position (NED)
		roll (float)		: roll angle
		pitch (float)		: pitch angle
		yaw (float)			: yaw angle

	convariance:
	Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle 
	(states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five 
	entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
	"""
	print("SENDING: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f" 
		% (xyz[0], xyz[1], xyz[2], cb[0], cb[1], cb[2]))
	master.mav.vision_position_estimate_send(
		int(cb[3]),	
		xyz[0],
		xyz[1],
		xyz[2],
		cb[0],
		cb[1],
		cb[2])

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(value):
	# print("time %d" % (value.time_boot_ms))
	# # print(value)
	# print("roll %f" % (value.roll))
	# print("pitch %f" % (value.pitch))
	# print("yaw %f" % (value.yaw))
	cb_vals[0] = value.roll
	cb_vals[1] = value.pitch
	cb_vals[2] = value.yaw
	cb_vals[3] = value.time_boot_ms

def marvelmind_loop():
	#enter address value for each drone here?
	print("ATTEMPTING TO CONNECT TO HEDGE")
	hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) 
	# create MarvelmindHedge thread
	hedge.start()
	print("HEDGE FOUND AND CONNECTED!")

	data = np.zeros(3, dtype=float)
	while True:
		try:
			hedge.dataEvent.wait(1)
			hedge.dataEvent.clear()
			if (hedge.positionUpdated):

				# hedge.print_position()
				pos = hedge.position()
				#possibly need NED conversion here
				data[0] = pos[1] #x
				data[1] = pos[2] #y
				data[2] = -(pos[3]) #z
				q1.put(data)
		except KeyboardInterrupt:
			hedge.stop()
			sys.exit()
		
def listener_thread(master, callbacks):
	interesting_messages = list(callbacks.keys())
	# time_after_loop = time.time() # initialization
	# frequency = 0.30
	# main programm
	data = np.zeros(4, dtype=float)
	while not thread_should_exit:
		# time_before_loop = time.time()
		# if time_before_loop - time_after_loop >= frequency:
		# 	real_frequency = time_before_loop - time_after_loop
		# 	print(real_frequency)
		# 	time_after_loop = time.time()
		# send a heartbeat msg
		# current_time_us = int(round(time.time() * 1000000))
		# master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
		# 						mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
		# 						0,
		# 						0,
		# 						0)
		m = master.recv_match(type=interesting_messages, blocking=False)
		if m is None:
			continue
		callbacks[m.get_type()](m)
		data[0] = cb_vals[0] #yaw pitch roll time
		data[1] = cb_vals[1]
		data[2] = cb_vals[2]
		data[3] = cb_vals[3]
		q2.put(data)

def mavlink_loop():
	time_after_loop = time.time() # initialization
	frequency = 0.30

	# main programm
	old_pos = np.zeros(3, dtype=float)
	old_ypr = np.zeros(4, dtype=float)
	while not thread_should_exit:
		time_before_loop = time.time()
		if time_before_loop - time_after_loop >= frequency:
			real_frequency = time_before_loop - time_after_loop
			print(real_frequency)
			time_after_loop = time.time()
		# send a heartbeat msg
		# current_time_us = int(round(time.time() * 1000000))
		# master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
		# 						mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
		# 						0,
		# 						0,
		# 						0)

		if (q1.empty() == False):
			old_pos = q1.get()
			print("NEW UPDATED BEACON VALUE")
		if (q2.empty() == False):
			old_ypr = q2.get()
			print("NEW UPDATED IMU VALUE")
		# old_val[0] = 0.12
		# old_val[1] = 0.23
		# old_val[2] = 0.0
		send_gps_data(old_ypr, old_pos)
		time.sleep(0.025)

home_lat = int(52.426595 * 10000000.0)   # Somewhere random
home_lon = int(10.786546 * 10000000.0)	# Somewhere random
home_alt = 64       # Somewhere random

current_time_us = 0

master = mavutil.mavlink_connection('udpin:127.0.0.1:15667', baud=57600)
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

mavlink_callbacks = {
	'ATTITUDE': att_msg_callback,
}

set_default_global_origin()
set_default_home_position()
thread_should_exit = False

cb_vals = np.zeros(4, dtype=float)

q1 = queue.Queue()
q2 = queue.Queue()

marvelmind_thread = threading.Thread(target=marvelmind_loop)
callback_thread = threading.Thread(target=listener_thread, args=(master, mavlink_callbacks))
mavlink_thread = threading.Thread(target=mavlink_loop)

marvelmind_thread.start()
callback_thread.start()
mavlink_thread.start()


time.sleep(300)
thread_should_exit = True
print('Closing the script...')

thread_should_exit = True
mavlink_thread.join()
marvelmind_thread.join()



"""
x + CAL_ACC0_ID [16,49] : 1310988
x + CAL_ACC0_PRIO [17,50] : 50
x + CAL_ACC0_XOFF [19,52] : -0.0118
x + CAL_ACC0_YOFF [21,54] : 0.0023
x + CAL_ACC0_ZOFF [23,56] : 0.0505
x + CAL_ACC1_ID [25,58] : 1310996
x + CAL_ACC1_PRIO [26,59] : 50
x + CAL_ACC1_XOFF [28,61] : 0.2037
x + CAL_ACC1_YOFF [30,63] : 0.0485
x + CAL_ACC1_ZOFF [32,65] : 0.4000
x + CAL_ACC2_ID [34,67] : 1311004
x + CAL_ACC2_PRIO [35,68] : 50
x + CAL_ACC2_XOFF [37,70] : -0.0150
x + CAL_ACC2_YOFF [39,72] : 0.0042
x + CAL_ACC2_ZOFF [41,74] : 0.0482
x + CAL_GYRO0_ID [51,100] : 1310988
x + CAL_GYRO1_ID [57,106] : 1310996
x + CAL_GYRO2_ID [63,112] : 1311004
x + CAL_MAG0_ID [70,124] : 197388
x + CAL_MAG1_ID [85,139] : 197644
x + COM_FLIGHT_UUID [137,428] : 7
x + EKF2_AID_MASK [191,483] : 24
x + EKF2_HGT_MODE [235,527] : 3
x + EKF2_MAG_DECL [242,534] : 3.0805
x + IMU_INTEG_RATE [358,786] : 250
x + LND_FLIGHT_T_LO [366,803] : 118988000
x + PWM_AUX_OUT [505,1055] : 1234
x + PWM_MAIN_OUT [524,1146] : 1234
x + SENS_BOARD_X_OFF [676,1344] : 0.0000
x + SENS_DPRES_OFF [679,1348] : 0.0010
x + SYS_AUTOSTART [691,1403] : 10016
`
"""
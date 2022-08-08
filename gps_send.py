from pymavlink import mavutil
from marvelmind import MarvelmindHedge
import sys
import threading
import time
import numpy as np
from numpy import NaN

def send_gps_data(q, x, y, z):
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
	print("SENDING: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f" % (x, y, z, q[0], q[1], q[2]))
	master.mav.vision_position_estimate_send(
		int(q[3]),
		x,
		y,
		z,
		q[0],
		q[1],
		q[2])

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(value):
	# print("time %d" % (value.time_boot_ms))
	# print(value)
	# print("roll %f" % (value.roll))
	# print("pitch %f" % (value.pitch))
	# print("yaw %f" % (value.yaw))
	# print("q4 %f" % (value.q4))
	q_array[0] = value.roll
	q_array[1] = value.pitch
	q_array[2] = value.yaw
	q_array[3] = value.time_boot_ms




def mavlink_loop(master, callbacks):
	'''a mai	# print("q1 %f" % (value.q1))
	# print("q2 %f" % (value.q2))
	# print("q3 %f" % (value.q3))
	# print("q4 %f" % (value.q4))
	# q_array[0] = value.q1
	# q_array[1] = value.q2
	# q_array[2] = value.q3
	# q_array[3] = value.q4n routine for a thread; reads data from a mavlink connection,
	calling callbacks based on message type received.
	'''
	xx = 0.0
	yy = 0.0
	zz = 0.0
	interesting_messages = list(callbacks.keys())
	while not mavlink_thread_should_exit:
		# send a heartbeat msg
		# current_time_us = int(round(time.time() * 1000000))
		master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
								mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
								0,
								0,
								0)
		m = master.recv_match(type=interesting_messages, timeout=0.1, blocking=True)
		if m is None:
			continue
		callbacks[m.get_type()](m)
		# hedge.dataEvent.wait()
		# hedge.dataEvent.clear()
		# try:
		# if (hedge.positionUpdated):
		# 	data = hedge.position()
		# 	xx = data[1]
		# 	yy = data[2]
		# 	zz = data[3]
		# except:
			# pass
			# print("time: %f x: %f y: %f z: %f" % (data[5], xx, yy, zz))
		# NEED TO CHECK IF THE NED TRANSLATION IS NEEDED HERE
		send_gps_data(q_array, xx, yy, -(zz))
		# time.sleep(0.001)

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

#enter address value for each drone here?
# hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False, baud=500000)
# hedge.start()
q_array = np.zeros(4, dtype=float)

home_lat = int(52.426595 * 10000000.0)   # Somewhere random
home_lon = int(10.786546 * 10000000.0)	# Somewhere random
home_alt = 64       # Somewhere random

current_time_us = 0

master = mavutil.mavlink_connection('udpin:127.0.0.1:15667', baud=57600)
# master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

mavlink_callbacks = {
	'ATTITUDE': att_msg_callback,
}

set_default_global_origin()
# set_default_home_position()
mavlink_thread_should_exit = False
mavlink_thread = threading.Thread(target=mavlink_loop, args=(master, mavlink_callbacks))
mavlink_thread.start()

time.sleep(300)
mavlink_thread_should_exit = True
print('Closing the script...')

mavlink_thread_should_exit = True
mavlink_thread.join()




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
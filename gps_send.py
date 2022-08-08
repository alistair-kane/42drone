from pymavlink import mavutil
from marvelmind import MarvelmindHedge
import sys
import threading
import time
import numpy as np
from numpy import NaN

def send_gps_data(time, q, x, y, z):
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
		np.array([q[0], q[1], q[2], q[3]], dtype=float),
		x,
		y,
		z)

#enter address value for each drone here?
# hedge = MarvelmindHedge(tty = "/dev/ttyUSB1", adr=None, debug=False)
# hedge.start()
time_v = 0
q_array = np.zeros(4, dtype=float)

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(value):
	print("time %d" % (value.time_boot_ms))
	print("q1 %f" % (value.q1))
	print("q2 %f" % (value.q2))
	print("q3 %f" % (value.q3))
	print("q4 %f" % (value.q4))
	time_v = value.time_boot_ms
	q_array[0] = value.q1
	q_array[1] = value.q2
	q_array[2] = value.q3
	q_array[3] = value.q4




def mavlink_loop(conn, callbacks):
	'''a main routine for a thread; reads data from a mavlink connection,
	calling callbacks based on message type received.
	'''
	interesting_messages = list(callbacks.keys())
	while not mavlink_thread_should_exit:
		# send a heartbeat msg
		conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
								mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
								0,
								0,
								0)
		m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
		if m is None:
			continue
		callbacks[m.get_type()](m)
		send_gps_data(time_v, q_array, 7, 7, 0)
		time.sleep(0.01)

# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
    conn.mav.set_gps_global_origin_send(
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

    conn.mav.set_home_position_send(
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

home_lat = 151269321    # Somewhere random
home_lon = 16624301     # Somewhere random
home_alt = 163000       # Somewhere random

master = mavutil.mavlink_connection('udpin:127.0.0.1:15667', baud=57600)
# master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

mavlink_callbacks = {
	'ATTITUDE_QUATERNION': att_msg_callback,
}

set_default_global_origin()
set_default_home_position()
mavlink_thread_should_exit = False
mavlink_thread = threading.Thread(target=mavlink_loop, args=(master, mavlink_callbacks))
mavlink_thread.start()

time.sleep(20)
mavlink_thread_should_exit = True
print('Closing the script...')

mavlink_thread_should_exit = True
mavlink_thread.join()
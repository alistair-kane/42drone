from numpy import NaN, uint64
from pymavlink import mavutil
import sys
import time
import numpy as np

FREQ = 30
CYCLETIME = 1/FREQ

def print_direction(xo, yo, zo, x, y, z):
	if (abs(x - xo) > abs(y - yo)):
		if (x > xo):
			print("going north")
		else:
			print("going south\n")
		if (z > zo):
			print("going down\n")
		else:
			print("going up\n")
	else:
		if (y > yo):
			print("going east")
		else:
			print("going west\n")
		if (z > zo):
			print("going down\n")
		else:
			print("going up\n")

# https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP
# def send_vision_position_estimate_message(x: float, y: float, z: float):
def send_vision_position_estimate_message(x: float, y: float, z: float):
	q = np.zeros(4, dtype=float)
	# Send the message
	master.mav.att_pos_mocap_send(
		int(time.time()),            # us Timestamp (UNIX time or time since system boot)
		q,
        x,     # X position (NED)
	    y,     # Y position (NED)
		z,    # Z position (NED)
		np.zeros(21, dtype=float)
	)

class sendGPS():
	def __init__(self):
		self.running = True
		#can filter the beacon address here?

	def run(self):
		x_old = 0.0
		y_old = 0.0 
		z_old = 0.0

		# t0 = time.perf_counter()  # Time ref point
		# time_counter = t0  # Will be incremented with CYCLETIME for each iteration
		while self.running:
			try:
					# THE NED TRANSLATION IS NEEDED HERE
					y = 0 # x should increase + when going north
					x = 0 # y should increase + when going east
					z = 0.1 # z should increase + when going down
					# x_old = x
					# y_old = y
					# z_old = z
					send_vision_position_estimate_message(x, y, z)
					print("sent gps message t: %f | x: %f | y: %f | z: %f" % (time.time(), x, y, z))

				# now = time.perf_counter()
				# elapsed_time = now - t0
				# target_time = time_counter + CYCLETIME
				# print("sleeping")
				# if elapsed_time < target_time:
					# time.sleep(target_time - elapsed_time)
				
				# time_counter += CYCLETIME
			except KeyboardInterrupt:
				self.hedge.stop()
				self.running = False
			time.sleep(0.02)
	
	def stop(self):
		self.running = False
		sys.exit()

def main():
	global master
	# master = mavutil.mavlink_connection("/dev/ttyUSB1", baud=57600)
	master = mavutil.mavlink_connection('udpin:127.0.0.1:15667', baud=57600)
	master.wait_heartbeat()
	print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
	gps = sendGPS()
	gps.run()

main()
from pymavlink import mavutil
from marvelmind import MarvelMindHedge
import sys

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

hedge = MarvelMindHedge(tty = "/dev/ttyUSB1", adr=None, debug=False)
hedge.start()

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

while True:
	try:
		hedge.dataEvent.wait(1)
		hedge.dataEvent.clear()
		if (hedge.fusionImuUpdated)
			hedge.print_imu_fusion()
	except KeyboardInterrupt:
		hedge.stop()
		sys.exit()
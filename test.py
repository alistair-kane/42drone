from pymavlink import mavutil
from marvelmind import MarvelMindHedge
import time
import sys

# hedge = MarvelMindHedge(tty = "/dev/ttyUSB1", adr=None, debug=False)
# hedge.start()

master = mavutil.mavlink_connection("udpin:127.0.0.1:15667", baud=57600)


master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))


master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
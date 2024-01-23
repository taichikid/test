import time
import math
from pymavlink import mavutil
import ctypes
import threading
import serial

# ser = serial.Serial('COM4', 57600, timeout=1)
# master = mavutil.mavlink_connection('COM4',force_connected=True)
# master = mavutil.mavlink_connection(ser)
ser.wait_heartbeat()
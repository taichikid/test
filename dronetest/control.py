import time
import math
from pymavlink import mavutil
import ctypes
import threading
import serial

class CustomThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return
    
    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id
    
    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

#座標情報出力
def check_gpi(master):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            print(msg)
        time.sleep(5)

#命令送信
def send_msg_rc(master, roll=0, pitch=0, throttle=0, yaw=0):
    while True:
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component, 
            roll, pitch, throttle, yaw, 0, 0, 0, 0) # R, P, Th, Y
        time.sleep(0.2)

#
def print_msg_AHRS2(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="AHRS2", blocking=True)
    print(msg)

#下向きLiDARの情報出力
def print_msg_DISTANCE_SENSOR(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        0,0,0,0,0,0
    )

    master.mav.send(msg)
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True)
    # print(msg)
    # print(msg.current_distance)
    distance = msg.current_distance / 100
    print(distance)
    return distance

def check_direction_pi(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="AHRS2", blocking=True)
    yaw = msg.yaw
    degrees = math.degrees(yaw)
    print(degrees)
    return degrees

#着陸
def land(vehicle):
    msg = vehicle.mav.command_long_encode(
        vehicle.target_system, vehicle.target_component,   # target system, target component
        mavutil.mavlink.MAV_CMD_NAV_LAND, # frame
        0,
        0,0,0,0,0,0,0
    )

    vehicle.mav.send(msg)
    msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

def mode_stab(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

def command(c, th_send_msg_rc=None):
    flag_send_msg_rc = False
    if c == "a":
        c = "idle"
        print("ARM")
        master.arducopter_arm()
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        print(f"result = {msg.result}")
    
    if c == "d":
        c = "idle"
        print("DISARM")
        master.arducopter_disarm()
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        print(f"result = {msg.result}")

    if c == "up":
        throttle = 2000
        roll = 1500 # お試し
        pitch = 1500
        yaw = 1500
        flag_send_msg_rc = True
    if c == "hov":
        throttle = 1625
        roll = 1500 # お試し
        pitch = 1500
        yaw = 1500
        flag_send_msg_rc = True
    if c == "land":
        land(master)
        # throttle = 1600
        # roll = 1500 # お試し
        # pitch = 1500
        # yaw = 1500
        #flag_send_msg_rc = True
    if c == "stab":
        mode_stab(master)
    if c == "idle":
        roll = 1500
        pitch = 1500
        throttle = 0
        yaw = 1500
        flag_send_msg_rc = True
    if c == "right":
        roll = 2000
        flag_send_msg_rc = True
    if c == "left":
        roll = 1000
        flag_send_msg_rc = True
    if c == "forward":
        pitch = 1000
        flag_send_msg_rc = True
    if c == "back":
        pitch = 2000    
        flag_send_msg_rc = True
    if c == "turnr":
        yaw = 1600
        flag_send_msg_rc = True
    if c == "turnl":
        yaw = 1400
        flag_send_msg_rc = True

    if c == "pa":
        print_msg_AHRS2(master)
    if c == "pd":
        print_msg_DISTANCE_SENSOR(master)

    if flag_send_msg_rc:
        if th_send_msg_rc is not None:
            th_send_msg_rc.raise_exception()
            th_send_msg_rc.join()
            th_send_msg_rc = None
        if roll + pitch + throttle + yaw > 0:
            th_send_msg_rc = CustomThread(
                name="send_msg_rc", target=send_msg_rc, 
                args=(master,),
                kwargs={"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw})
            th_send_msg_rc.start()
        else:
            master.mav.rc_channels_override_send(
                master.target_system, 
                master.target_component, 
                0, 0, 0, 0, 0, 0, 0, 0)
        
        flag_send_msg_rc = False

    return th_send_msg_rc

def commands(master, th_send_msg_rc, dict_params):
    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
        th_send_msg_rc = None
    th_send_msg_rc = CustomThread(
        name="send_msg_rc", target=send_msg_rc, 
        args=(master,),
        kwargs=dict_params)
    th_send_msg_rc.start()
    return th_send_msg_rc

try:
    # SITLへの接続
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    master.wait_heartbeat()

    # th_check_gpi = CustomThread(name="check_gpi", target=check_gpi, args=(master,))
    # th_check_gpi.start()

    flag_send_msg_rc = False
    th_send_msg_rc = None
    dict_params = {"roll": 1500, "pitch": 1500, "throttle": 0, "yaw": 1500}

    #起動arm
    th_send_msg_rc = command("a")
    time.sleep(1)

    #上昇takeoff
    dict_params["throttle"] = 2000
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    while print_msg_DISTANCE_SENSOR(master) < 50:
        time.sleep(0.1)
        pass

    dict_params["throttle"] = 1620
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    count = 0
    target_degrees = 150
    for i in range(20):
        current_degrees = check_direction_pi(master)
        diff = abs(current_degrees - target_degrees)
        if diff > 30:
            power = 200
        if diff > 20:
            power = 125
        elif diff > 7:
            power = 35
        else:
            power = 22
        
        if current_degrees > target_degrees:
            power *= -1
        
        dict_params["yaw"] = 1500 + power
        th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
        time.sleep(0.5)
    dict_params["yaw"] = 1500
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    dict_params["pitch"] = 1000
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
    time.sleep(10)

    dict_params["pitch"] = 1500
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
    
    dict_params["throttle"] = 0
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    while print_msg_DISTANCE_SENSOR(master) > 20:
        time.sleep(0.1)
        pass

    # dict_params["throttle"] = 2000
    # th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
    # time.sleep(0.5)

    # th_send_msg_rc = command("land", th_send_msg_rc)

    th_send_msg_rc.raise_exception()
    th_send_msg_rc.join()

    exit(1)
    
    while print_msg_DISTANCE_SENSOR(master) > 20:
        th_send_msg_rc = command("idle", th_send_msg_rc)
    th_send_msg_rc = command("up", th_send_msg_rc)
    time.sleep(0.5)
    th_send_msg_rc = command("land", th_send_msg_rc)
    

    # count = 0
    # target_degrees = 45
    # for i in range(50):
    #     current_degrees = check_direction_pi(master)
    #     if current_degrees > target_degrees:
    #         dict_params["yaw"] = 1450
    #     else:
    #         dict_params["yaw"] = 1550
    #     th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
    #     time.sleep(0.5)

    # dict_params["yaw"] = 1500
    # th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)
    # check_direction_pi(master)
    

    print("test done")

    # th_send_msg_rc = command("up", th_send_msg_rc)
    # while True:
    #     if print_msg_DISTANCE_SENSOR(master) > 100:
    #         while print_msg_DISTANCE_SENSOR(master) > 20:
    #             th_send_msg_rc = command("idle", th_send_msg_rc)
    #         th_send_msg_rc = command("up", th_send_msg_rc)
    #         time.sleep(0.5)
    #         th_send_msg_rc = command("land", th_send_msg_rc)
    #         break
    #     time.sleep(0.2)

    # # #前進forward
    # # command("forward")

    # # #着陸land
    # # command("land")

    # #常時情報出力
    # # if th_send_msg_rc is not None:
    # #     th_send_msg_rc.raise_exception()
    # #     th_send_msg_rc.join()
    # # th_check_gpi.raise_exception()
    # # th_check_gpi.join()
    # while True:
    #     if print_msg_DISTANCE_SENSOR(master) < 1:
    #         # 接続を閉じる
    #         master.close()

    # th_send_msg_rc = command("stab", th_send_msg_rc)

    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
    # th_check_gpi.raise_exception()
    # th_check_gpi.join()
except KeyboardInterrupt:
    try:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
    except:
        pass

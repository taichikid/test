import time
from pymavlink import mavutil
import ctypes
import threading

import struct
import numpy as np
import serial
from serial.tools import list_ports

from server import recv_takeoff

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

def send_msg_rc(master, dict_rc):
    while True:
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component, 
            dict_rc["roll"], dict_rc["pitch"], dict_rc["throttle"], dict_rc["yaw"],
            0, 0, 0, 0) # R, P, Th, Y
        time.sleep(0.2)

def get_roll_pitch_AHRS2(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="AHRS2", blocking=True)
    return msg.roll, msg.pitch

def print_msg_DISTANCE_SENSOR(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        0,0,0,0,0,0
    )

    master.mav.send(msg)
    #res = master.recv_match(blocking=True)
    #print(res)

    res = master.recv_match(type="DISTANCE_SENSOR", blocking=True)
    #print(res.current_distance * 0.01)
    return res.current_distance * 0.01

def set_rc(dict_rc, th_send_msg_rc):
    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
        th_send_msg_rc = None
    th_send_msg_rc = CustomThread(
        name="send_msg_rc", target=send_msg_rc, 
        args=(master, dict_rc),
        kwargs={})
    th_send_msg_rc.start()
    return th_send_msg_rc

def mode_stab(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
        0,1,0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

def mode_land(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0,0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

def disarm_force(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,0,21196,0,0,0,0,0
    )
    master.mav.send(msg)
    #msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

class HovController():
    def __init__(self, current_alt, throttle=1270, min_throttle=1200, 
                 max_throttle=1280, value_t=10, error=0.1, width=1):
        self.current_alt = current_alt
        self.pre_alt = self.current_alt
        self.current_v = self.current_alt - self.pre_alt
        self.pre_v = self.current_v
        self.current_a = self.current_v - self.pre_v

        self.throttle = throttle
        self.min_throttle = min_throttle
        self.max_throttle = max_throttle
        self.value_t = value_t
        self.error = error
        self.width = width

    def get_throttle(self, current_alt, target_alt, error_coef_1=10, 
                     error_coef_2=4, width_coef_1=4, width_coef_2=2):
        self.pre_alt = self.current_alt
        self.current_alt = current_alt
        self.pre_v = self.current_v
        self.current_v = self.current_alt - self.pre_alt
        self.current_a = self.current_v - self.pre_v

        future_alt = self.current_alt + self.current_v * self.value_t + \
                     0.5 * self.current_a * self.value_t ** 2
        if future_alt > target_alt + self.error * error_coef_1:
            self.throttle -= self.width * width_coef_1
        elif future_alt > target_alt + self.error * error_coef_2:
            self.throttle -= self.width * width_coef_2
        elif future_alt > target_alt + self.error:
            self.throttle -= self.width
        elif future_alt < target_alt - self.error * error_coef_1:
            self.throttle += self.width * width_coef_1
        elif future_alt < target_alt - self.error * error_coef_2:
            self.throttle += self.width * width_coef_2
        elif future_alt < target_alt - self.error:
            self.throttle += self.width

        if self.throttle > self.max_throttle:
            self.throttle = self.max_throttle
        elif self.throttle < self.min_throttle:
            self.throttle = self.min_throttle
        return int(self.throttle)

def get_lidar_frames_from_buffer(buffer):
    frames = []
    
    while len(buffer) >= 47:
        # ヘッダーが正しくない場合、次のヘッダーを探す
        if buffer[0] != 0x54:
            if 0x54 in buffer:
                buffer = buffer[buffer.index(0x54):]
            else:
                break
        
        # 一つのフレームを取り出す
        frame_data = buffer[:47]
        buffer = buffer[47:]
        
        # フレームのデータが正しいか確認
        if not check_lidar_frame_data(frame_data):
            continue
        
        frames.append(get_lidar_frame(frame_data))
    
    return frames, buffer

def check_lidar_frame_data(data):
    return data[1] == 0x2C and len(data) == 47

def get_lidar_frame(data):
    frame = {}
    frame['header'] = data[0]
    frame['ver_len'] = data[1]
    frame['speed'] = struct.unpack("<H", bytes(data[2:4]))[0]
    frame['startAngle'] = struct.unpack("<H", bytes(data[4:6]))[0]
    
    points = []
    for i in range(12):
        start_index = 6 + 3 * i
        distance = struct.unpack("<H", bytes(data[start_index:start_index+2]))[0]
        intensity = data[start_index+2]
        points.append((distance, intensity))
    
    frame['points'] = points
    frame['endAngle'] = struct.unpack("<H", bytes(data[42:44]))[0]
    frame['timestamp'] = struct.unpack("<H", bytes(data[44:46]))[0]
    frame['crc8'] = data[46]
    
    return frame

if __name__ == "__main__":
    ################################################################
    # 起動
    ################################################################
    
    # 接続開始
    master = mavutil.mavlink_connection('/dev/ttyUSB0')
    master.wait_heartbeat()

    # モードをSTABILIZEに変更
    master.set_mode(master.mode_mapping()["STABILIZE"])

    # roll、pitchを補正
    base_roll = 1462 #1500
    base_pitch = 1500 #1500
    ahrs_roll, ahrs_pitch = get_roll_pitch_AHRS2(master)
    # print(ahrs_roll, ahrs_pitch)
    #base_roll = int(base_roll + 1500 * ahrs_roll)
    #base_pitch = int(base_pitch + 1500 * ahrs_pitch)

    #width = 500 / 1.5
    #base_roll = int(base_roll + width * ahrs_roll)
    #base_pitch = int(base_pitch + width * ahrs_pitch)
    #print(base_roll, base_pitch)

    # 指令値を初期化
    th_send_msg_rc = None
    dict_rc = {}
    dict_rc["roll"] = base_roll
    dict_rc["pitch"] = base_pitch
    dict_rc["throttle"] = 0
    dict_rc["yaw"] = 1500
    th_send_msg_rc = set_rc(dict_rc, th_send_msg_rc)

    # 障害物回避用
    drone_size = 800
    avoid_distance = 0.5 * drone_size + 2000 #安全距離
    safe_distance = 0.5 * drone_size + 1000
    power_roll = 80
    power_pitch = 0
    power_max = 50
    direction = 0 # 進行方向（前=0, 右=1，左=-1）
    ser = serial.Serial('/dev/ttyUSB0', 230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    # recv_takeoff()

    # ARM
    timeout = 3
    start_time = time.time()
    while True:
        master.arducopter_arm()
        res = master.recv_match(type="COMMAND_ACK", blocking=True)
        if res.result == 0:
            break
        time.sleep(0.1)
        
        if time.time() - start_time > timeout:
            print("ARM TIMEOUT")
            exit()
    
    try:
        ################################################################
        # 自律飛行
        ################################################################

        total_time = 12
        max_safe_alt = 3
        target_alt = 0.5
        
        start_alt = print_msg_DISTANCE_SENSOR(master)
        hov = HovController(start_alt)

        start_time = time.time()
        while time.time() - start_time < total_time:
            current_alt = print_msg_DISTANCE_SENSOR(master)

            if current_alt > max_safe_alt:
                print("emargency land")
                break

            throttle = hov.get_throttle(current_alt, target_alt)

            # 障害物回避
            buffer = bytearray()
            all_frames = []
            list_data = []
            while ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
            frames, buffer = get_lidar_frames_from_buffer(buffer)
            if frames:
                all_frames.extend(frames)
            for frame in all_frames:
                diffAngle = (frame['endAngle'] - frame['startAngle']) / 11.0 if frame['endAngle'] > frame['startAngle'] else (frame['endAngle'] + 36000.0 - frame['startAngle']) / 11.0
                datas = []
                for i, (distance, intensity) in enumerate(frame['points']):
                    angle = (frame['startAngle'] + i * diffAngle) * (np.pi / 18000.0)
                    angle = angle % (2 * np.pi)
                    y = distance * np.cos(angle)
                    x = distance * np.sin(angle)
                    degree = np.degrees(angle)
                    data = [degree,distance,x,y]
                    datas.append(data)
                list_data.extend(datas)
            
            np_list_data = np.array(list_data)
            
            nda_theta = np_list_data[:,0] #angle
            nda_distance = np_list_data[:,1] #distance
            nda_x = np_list_data[:,2] #x_distance
            nda_y = np_list_data[:,3] #y_distance
            
            #距離が0以下の障害物が全体の50分の１以上になったら緊急回避
            if np.count_nonzero(nda_distance < 0) > len(nda_distance) / 50: 
                print("butukaruzo! hanten!")
                danger_degree = nda_theta[np.argmin(nda_distance)] #距離が最小値の角度
                go_degree = (danger_degree + 180) % 360
                go_radian = np.radians(go_degree)
                pitch = int(base_pitch - power_max * np.cos(go_radian))
                roll = int(base_roll + power_max * np.sin(go_radian))
                distance = 0
            else:
                #
                if np.count_nonzero((-1.0 * safe_distance < nda_x) & (nda_x < safe_distance) & (drone_size * 0.5 < nda_y) & (nda_y < avoid_distance)) > len(nda_distance) / 50:
                        pitch = base_pitch
                        if direction > 0: # 右に飛行中
                                if np.count_nonzero((drone_size * 0.5 < nda_x) & (nda_x < avoid_distance) & (-1.0 * safe_distance < nda_y) & (nda_y < safe_distance)) > len(nda_distance) / 50:
                                        print("right -> left")
                                        roll = base_roll - power_roll
                                        direction = -1
                                else:
                                        print("right")                      
                                        roll = base_roll + power_roll  
                        elif direction < 0: # 左に飛行中
                                if np.count_nonzero((-1 * avoid_distance < nda_x) & (nda_x < drone_size * -0.5) & (-1.0 * safe_distance < nda_y) & (nda_y < safe_distance)) > len(nda_distance) / 50:
                                        print("left -> right")
                                        roll = base_roll + power_roll
                                        direction = 1
                                else:
                                        print("left")
                                        roll = base_roll - power_roll
                        else: # 前に飛行中
                                print("syougaibutu")
                                if np.count_nonzero((-1 * avoid_distance < nda_x) & (nda_x < drone_size * -0.5) & (-1.0 * safe_distance < nda_y) & (nda_y < avoid_distance)) > len(nda_distance) / 50:
                                    print("right")
                                    roll = base_roll + power_roll
                                    direction = 1
                                elif np.count_nonzero((drone_size * 0.5 < nda_x) & (nda_x < avoid_distance) & (-1.0 * safe_distance < nda_y) & (nda_y < avoid_distance)) > len(nda_distance) / 50:
                                    print("left")
                                    roll = base_roll - power_roll
                                    direction = -1
                else:
                        print("forward")
                        pitch = base_pitch - power_pitch
                        roll = base_roll
                        direction = 0
            
            #　指令
            dict_rc["throttle"] = throttle
            dict_rc["pitch"] = pitch
            dict_rc["roll"] = roll
            print(dict_rc)
            th_send_msg_rc = set_rc(dict_rc, th_send_msg_rc)
    except:
        pass
   
    ################################################################
    # 終了
    ################################################################

    # 着陸
    master.set_mode(master.mode_mapping()["LAND"])
    while current_alt >= start_alt:
        print(current_alt, ">", start_alt)
        current_alt = print_msg_DISTANCE_SENSOR(master)
        time.sleep(0.04)

    # 指令値をリセット
    dict_rc["roll"] = base_roll
    dict_rc["pitch"] = base_pitch
    dict_rc["throttle"] = 0
    dict_rc["yaw"] = 1500
    th_send_msg_rc = set_rc(dict_rc, th_send_msg_rc)

    # DISARM
    disarm_force(master)

    # 片付け
    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()

    # 接続を閉じる
    master.close()


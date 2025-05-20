import serial
import struct
import threading
import time
import math
from multiprocessing import Process, Queue, Event

import displacement
import ultrasonic
import imu
import flowmeter
import servo
import load
import switch
from motor1 import motor as motor1
from motor2 import motor as motor2
import charger

# 串口参数
SERIAL_PORT = '/dev/ttyCH9344USB0'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE

# 控制参数
PI = math.pi
MOTOR1_SPEED_STOP = 0.0
MOTOR2_SPEED_STOP = 0.0
MOTOR1_SPEED_START = 3.0 / PI * 180
MOTOR2_SPEED_START = 3.0 / PI * 180
MOTOR1_CURRENT_STOP = 0.0
MOTOR2_CURRENT_STOP = 0.0
MOTOR1_CURRENT_START = 0.7
MOTOR2_CURRENT_START = 0.7
IMPULSE_DURATION = 5  # 点动保持周期数

# 周期任务频率（秒）10Hz
PERIODIC_INTERVAL = 0.1
# 周期补偿
# PERIODIC_COMP = -0.099

# 11.4 16.1823
# 10.8 15.3221
# 10.1 14.2645
# 2.2  7.1265（最低限位）
# 12.6 17.3713（最高TBD）

# 重力常数
G = 9.8

# 电机保护参数         
LOAD_WEIGHT_PROTECT = 250 / G        # 夹紧电机保护夹紧力（250 N）
LOAD_WEIGHT_PROTECT = 45 / G         # 夹紧电机保护夹紧力测试（45 N）
DISPLACEMENT_PROTECT_LOW = 7.5       # 位移传感器保护限位低
DISPLACEMENT_PROTECT_HIGH = 15.5     # 位移传感器保护限位高

MOTOR1_TPCB_PROTECT = 55.0           # 电机过热保护
MOTOR2_TPCB_PROTECT = 55.0
MOTOR1_TPCB_COOLDOWN = 5             # 温度下降5度重新启动
MOTOR2_TPCB_COOLDOWN = 5

# 电机过温保护测试，易于触发的参数
# MOTOR1_CURRENT_START = 0.9
# MOTOR2_CURRENT_START = 0.9
# MOTOR1_SPEED_START = 3 / PI * 180
# MOTOR2_SPEED_START = 3 / PI * 180
# MOTOR1_TPCB_PROTECT = 40.0
# MOTOR2_TPCB_PROTECT = 40.0

# 命令字常量
CMD_STATUS_UPLOAD = 0x0001
CMD_SERVO_CTRL    = 0x0002
CMD_MOTOR1_CTRL   = 0x0003
CMD_MOTOR2_CTRL   = 0x0004
CMD_MOTOR1_TEST   = 0x0005
CMD_MOTOR2_TEST   = 0x0006
CMD_CHARGER_ONOFF = 0x0007

# 帧结构常量
SYN1 = 0xAA
SYN2 = 0xBB
END = 0x55
PAYLOAD_LENGTH = 40
MAX_INCOMPLETE_COUNT = 2  # 可容忍2次周期未收齐指令帧

# 指令帧不全计数
incomplete_counter = 0


# 全局点动周期计数
motor1_impulse_counter = 0
motor2_impulse_counter = 0

# 串口对象
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, parity=PARITY, timeout=0)

def delay_precise(duration_s):
    start = time.perf_counter()
    end = start + duration_s
    while True:
        now = time.perf_counter()
        if now >= end:
            break

# 构造上传帧
def build_frame():
    # UNIX时间戳
    timestamp = int(time.time())
    
    payload = struct.pack('<9f4B2f2h3fi',
        displacement.displacement_distance,
        ultrasonic.uwc_distance,
        imu.imu_roll,
        imu.imu_pitch,
        imu.imu_yaw,
        flowmeter.flow_speed,
        flowmeter.flow_direction,
        servo.read_angle,
        load.weight * G,
        switch.switch_state,
        servo.servo_state,
        motor1.motor_status,
        motor2.motor_status,
        charger.out_voltage,
        charger.out_current,
        charger.chg_state,
        charger.error_code,
        charger.input_volt,
        charger.input_current,
        charger.sender_temperature,
        timestamp
    )
    cmd = struct.pack('<H', CMD_STATUS_UPLOAD)
    length = struct.pack('<H', len(payload))
    header = bytearray([SYN1, SYN2]) + cmd + length + payload
    checksum = sum(header) & 0xFF
    return header + bytearray([checksum, END])

# # 传感器队列和事件
# sensor_queues = {
#     'f': Queue(),  # 位移                                    0.04
#     'e': Queue(),  # E组（ultra, imu, flow, weight, switch） 0.038
#     'b': Queue(),  # 伺服                                    0.031
#     'c': Queue(),  # motor1                                  0.06
#     'd': Queue(),  # motor2                                  0.06
# }
# trigger_events = {
#     'f': Event(),
#     'e': Event(),
#     'b': Event(),
#     'c': Event(),
#     'd': Event(),
# }

# def worker_f(event, queue):
#     while True:
#         event.wait()
#         result = displacement.read_displacement()
#         queue.put({'data': result, 'success': displacement.read_success})
#         event.clear()

# def worker_e(event, queue):
#     while True:
#         event.wait()
#         result = {
#             'ultra': {'data': ultrasonic.read_ultrasonic_distance(), 'success': ultrasonic.read_success},
#             'imu': {'data': imu.read_imu_data(), 'success': imu.read_success},
#             'flow': {'data': flowmeter.read_flowmeter(), 'success': flowmeter.read_success},
#             'weight': {'data': load.read_weight_data(), 'success': load.read_success},
#             'switch': {'data': switch.read_switch_state(), 'success': switch.read_success},
#         }
#         queue.put({'data': result})
#         event.clear()

# def worker_b(event, queue):
#     while True:
#         event.wait()
#         result = servo.read_position()
#         queue.put({'data': result, 'success': servo.read_success})
#         event.clear()

# def worker_c(event, queue):
#     while True:
#         event.wait()
#         result = motor1.read_temperature()
#         queue.put({'data': result, 'success': motor1.read_success})
#         event.clear()

# def worker_d(event, queue):
#     while True:
#         event.wait()
#         result = motor2.read_temperature()
#         queue.put({'data': result, 'success': motor2.read_success})
#         event.clear()

# def start_sensor_workers():
#     Process(target=worker_f, args=(trigger_events['f'], sensor_queues['f']), daemon=True).start()
#     Process(target=worker_e, args=(trigger_events['e'], sensor_queues['e']), daemon=True).start()
#     Process(target=worker_b, args=(trigger_events['b'], sensor_queues['b']), daemon=True).start()
#     Process(target=worker_c, args=(trigger_events['c'], sensor_queues['c']), daemon=True).start()
#     Process(target=worker_d, args=(trigger_events['d'], sensor_queues['d']), daemon=True).start()

# # 统一采集所有设备状态
# def read_all_devices():

#     timestamps = []
#     timestamps.append(time.perf_counter())

#     # 触发所有采集任务
#     for evt in trigger_events.values():
#         evt.set()

#     timestamps.append(time.perf_counter())

#     # 获取所有结果
#     res_c = sensor_queues['c'].get() # 0.01
#     timestamps.append(time.perf_counter())

#     res_d = sensor_queues['d'].get() # 0.003
#     timestamps.append(time.perf_counter())
#     res_f = sensor_queues['f'].get() # 0.044
#     timestamps.append(time.perf_counter())

#     res_e = sensor_queues['e'].get() # 0.01
#     timestamps.append(time.perf_counter())

#     res_b = sensor_queues['b'].get() # 0.003
#     timestamps.append(time.perf_counter())

#     # 解析结果
#     displacement.read_success = res_f['success']
#     if displacement.read_success:
#         displacement.displacement_distance = res_f['data']

#     e = res_e['data']
#     ultrasonic.read_success = e['ultra']['success']
#     if ultrasonic.read_success:
#         ultrasonic.uwc_distance, ultrasonic.uwc_status, ultrasonic.uwc_temperature = e['ultra']['data']
#     imu.read_success = e['imu']['success']
#     if imu.read_success:
#         imu.imu_roll, imu.imu_pitch, imu.imu_yaw = e['imu']['data']
#     flowmeter.read_success = e['flow']['success']
#     if flowmeter.read_success:
#         flowmeter.flow_direction, flowmeter.flow_speed = e['flow']['data']
#     load.read_success = e['weight']['success']
#     if load.read_success:
#         load.weight = e['weight']['data']
#     switch.read_success = e['switch']['success']
#     if switch.read_success:
#         switch.switch_state = e['switch']['data']
    
#     servo.read_success = res_b['success'] 
#     if servo.read_success:
#         servo.read_angle = res_b['data']
    
#     motor1.read_success = res_c['success']
#     if motor1.read_success:
#         motor1.T_coil, motor1.T_pcb = res_c['data']
    
#     motor2.read_success = res_d['success']
#     if motor2.read_success:
#         motor2.T_coil, motor2.T_pcb = res_d['data']

#     timestamps.append(time.perf_counter())
#     # durations = [t1 - t0 for t0, t1 in zip(timestamps, timestamps[1:])]
#     # print("[read_all_devices]", " ".join(f"{d:>10.5f}" for d in durations))

#     # 打印状态
#     print("read_success:",
#         "disp", color_bool(displacement.read_success),
#         "ultr", color_bool(ultrasonic.read_success),
#         "imu ", color_bool(imu.read_success),
#         "flow", color_bool(flowmeter.read_success),
#         "load", color_bool(load.read_success),
#         "swit", color_bool(switch.read_success),
#         "sero", color_bool(servo.read_success),
#         "mot1", color_bool(motor1.read_success),
#         "mot2", color_bool(motor2.read_success)
#     )

def color_bool(value: bool) -> str:
    text = f"{value:>2}"  # 固定宽度为5，右对齐
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 电机保护逻辑判断
def motor1_motor2_protect():
    if switch.switch_state == 1 and motor1.motor_speed > 0:
        print("电机1 触发限位保护")
        motor1.motor_speed = MOTOR1_SPEED_STOP
        motor1.motor_current = MOTOR1_CURRENT_STOP

    if load.weight > LOAD_WEIGHT_PROTECT and motor1.motor_speed < 0:
        print("电机1 触发夹紧力保护")
        motor1.motor_speed = MOTOR1_SPEED_STOP
        motor1.motor_current = MOTOR1_CURRENT_STOP

    if displacement.displacement_distance < DISPLACEMENT_PROTECT_LOW and motor2.motor_speed > 0:
        print("电机2 触发下限位保护")
        motor2.motor_speed = MOTOR2_SPEED_STOP
        motor2.motor_current = MOTOR2_CURRENT_STOP

    if displacement.displacement_distance > DISPLACEMENT_PROTECT_HIGH and motor2.motor_speed < 0:
        print("电机2 触发上限位保护")
        motor2.motor_speed = MOTOR2_SPEED_STOP
        motor2.motor_current = MOTOR2_CURRENT_STOP

    if motor1.T_pcb > MOTOR1_TPCB_PROTECT and motor1.motor_enable == 0x01:
        print("电机1 触发过温保护")
        motor1.motor_enable = 0x00
        motor1.enable_motor()
    if motor1.T_pcb < MOTOR1_TPCB_PROTECT - MOTOR1_TPCB_COOLDOWN and motor1.motor_enable == 0x00:
        print("电机1 解除过温保护")
        motor1.motor_enable = 0x01
        motor1.enable_motor()

    # print(f"[motor1.T_pcb]: {motor1.T_pcb:.1f} [motor2.T_pcb]: {motor2.T_pcb:.1f}")
    if motor2.T_pcb > MOTOR1_TPCB_PROTECT and motor2.motor_enable == 0x01:
        print("电机2 触发过温保护")
        motor2.motor_enable = 0x00
        motor2.enable_motor()
    if motor2.T_pcb < MOTOR1_TPCB_PROTECT - MOTOR2_TPCB_COOLDOWN and motor2.motor_enable == 0x00:
        print("电机2 解除过温保护")
        motor2.motor_enable = 0x01
        motor2.enable_motor()

# 电机点动模式计数与停止动作
def handle_motor_impulses():
    global motor1_impulse_counter, motor2_impulse_counter

    if motor1_impulse_counter > 0:
        motor1_impulse_counter -= 1
        if motor1_impulse_counter == 0:
            motor1.motor_speed = MOTOR1_SPEED_STOP
            motor1.motor_current = MOTOR1_CURRENT_STOP

    if motor2_impulse_counter > 0:
        motor2_impulse_counter -= 1
        if motor2_impulse_counter == 0:
            motor2.motor_speed = MOTOR2_SPEED_STOP
            motor2.motor_current = MOTOR2_CURRENT_STOP

# # 电机队列和事件
# motor_events = {
#     'servo': Event(),
#     'motor1': Event(),
#     'motor2': Event()
# }

# # 从主进程读入参数
# motor_param_queues = {
#     'servo': Queue(),
#     'motor1': Queue(),
#     'motor2': Queue()
# }

# # 将状态传回主进程
# motor_status_queues = {
#     'motor1': Queue(),
#     'motor2': Queue()
# }

# def servo_worker(event, param_queue):
#     while True:
#         event.wait()
#         angle = param_queue.get()
#         servo.set_angle = angle
#         servo.set_position()
#         event.clear()

# def motor1_worker(event, param_queue, status_queue):
#     while True:
#         event.wait()
#         speed, current = param_queue.get()
#         motor1.motor_speed = speed
#         motor1.motor_current = current
#         motor1.set_speed_and_current()
#         status_queue.put(motor1.motor_status)
#         event.clear()


# def motor2_worker(event, param_queue, status_queue):
#     while True:
#         event.wait()
#         speed, current = param_queue.get()
#         motor2.motor_speed = speed
#         motor2.motor_current = current
#         motor2.set_speed_and_current()
#         status_queue.put(motor2.motor_status)
#         event.clear()


# def start_motor_workers():
#     Process(target=servo_worker, args=(motor_events['servo'], motor_param_queues['servo']), daemon=True).start()
#     Process(target=motor1_worker, args=(motor_events['motor1'], motor_param_queues['motor1'], motor_status_queues['motor1']), daemon=True).start()
#     Process(target=motor2_worker, args=(motor_events['motor2'], motor_param_queues['motor2'], motor_status_queues['motor2']), daemon=True).start()

# # 控制伺服 电机
# def motor_ctrl():

#     # timestamps = []
#     # timestamps.append(time.perf_counter())

#     # 发送控制参数
#     motor_param_queues['servo'].put(servo.set_angle)
#     motor_param_queues['motor1'].put((motor1.motor_speed, motor1.motor_current))
#     motor_param_queues['motor2'].put((motor2.motor_speed, motor2.motor_current))

#     # 激活控制任务
#     motor_events['servo'].set()
#     motor_events['motor1'].set()
#     motor_events['motor2'].set()

#     # 等待返回状态
#     motor1.motor_status = motor_status_queues['motor1'].get()
#     motor2.motor_status = motor_status_queues['motor2'].get()

#     # timestamps.append(time.perf_counter())
#     # durations = [t1 - t0 for t0, t1 in zip(timestamps, timestamps[1:])]
#     # print("[read_all_devices]", " ".join(f"{d:>10.5f}" for d in durations))

# 查找下一个可能的帧头位置（从索引1开始）
def find_next_frame_head(buffer):
    for i in range(1, len(buffer) - 1):
        if buffer[i] == SYN1 and buffer[i + 1] == SYN2:
            return i
    return len(buffer)  # 没有找到，返回末尾（整段舍弃）

# 接收串口数据并从缓存中提取有效帧（大端格式，稳健异常处理）
rx_buffer = bytearray()

# 全局或静态变量：等待次数计数器
def process_received_data():
    global rx_buffer, incomplete_counter
    rx_buffer += ser.read(64)  # 读取串口数据追加进缓存

    while len(rx_buffer) >= 9:
        # print(f"Rcve:    {' '.join(f'{x:02X}' for x in rx_buffer)}")
        # 最小有效帧长度 = 9 字节（含 1 字节 payload）

        # 1. 帧头检查
        if rx_buffer[0] != SYN1 or rx_buffer[1] != SYN2:
            pos = find_next_frame_head(rx_buffer)
            print(f"Invalid header, skipping {pos} byte(s)")
            rx_buffer = rx_buffer[pos:]
            continue

        # 2. 提取 CMD 和 LENGTH（使用大端序）
        cmd, length = struct.unpack('<HH', rx_buffer[2:6])

        # 3. payload 长度非法
        if length > 5:
            pos = find_next_frame_head(rx_buffer)
            print(f"Illegal LENGTH={length}, skipping {pos} byte(s)")
            rx_buffer = rx_buffer[pos:]
            continue

        # 4. 判断是否有完整帧数据
        frame_len = 6 + length + 2  # header + payload + checksum + end
        if len(rx_buffer) < frame_len:
            incomplete_counter += 1
            if incomplete_counter >= MAX_INCOMPLETE_COUNT:
                print(f"Incomplete frame timeout, skipping 1 byte.")
                rx_buffer = rx_buffer[1:]  # 丢掉一个字节重新找帧
                incomplete_counter = 0
            return  # 否则继续等待
        else:
            incomplete_counter = 0  # 一旦收齐，计数归零

        # 5. 检查帧尾
        if rx_buffer[frame_len - 1] != END:
            pos = find_next_frame_head(rx_buffer)
            print(f"Invalid END byte, skipping {pos} byte(s)")
            rx_buffer = rx_buffer[pos:]
            continue

        # 6. 校验和验证（从 SYN1 到 PAYLOAD 最后一字节）
        checksum = sum(rx_buffer[:6 + length]) & 0xFF
        if checksum != rx_buffer[6 + length]:
            pos = find_next_frame_head(rx_buffer)
            print(f"Checksum error, skipping {pos} byte(s)")
            rx_buffer = rx_buffer[pos:]
            continue

        # 7. 提取并处理 payload
        payload = rx_buffer[6:6 + length]
        print(f"[RECV] CMD=0x{cmd:04X}, LENGTH={length}, PAYLOAD = {' '.join(f'{x:02X}' for x in payload)}")
        parse_command(cmd, payload)

        # 8. 移除已处理帧
        rx_buffer = rx_buffer[frame_len:]

# 指令解析，仅赋值全局变量，不执行控制动作
def parse_command(cmd, payload):
    global motor1_impulse_counter, motor2_impulse_counter

    if cmd == CMD_SERVO_CTRL and len(payload) == 5:
        print("旋转电机控制指令")
        start_stop, target_angle = struct.unpack('<Bf', payload)
        servo.set_angle = servo.read_angle if start_stop == 0x00 else target_angle

    elif cmd == CMD_MOTOR1_CTRL and len(payload) == 2:
        print("锁紧电机控制指令")
        start_stop, direction = struct.unpack('<BB', payload)
        print("start_stop, direction", start_stop, direction)
        if start_stop == 0x00:
            motor1.motor_speed = MOTOR1_SPEED_STOP
            motor1.motor_current = MOTOR1_CURRENT_STOP
        else:
            motor1.motor_speed = MOTOR1_SPEED_START if direction == 0 else -MOTOR1_SPEED_START
            motor1.motor_current = MOTOR1_CURRENT_START

    elif cmd == CMD_MOTOR2_CTRL and len(payload) == 2:
        print("充电电机控制指令")
        start_stop, direction = struct.unpack('<BB', payload)
        print("start_stop, direction", start_stop, direction)
        if start_stop == 0x00:
            motor2.motor_speed = MOTOR2_SPEED_STOP
            motor2.motor_current = MOTOR2_CURRENT_STOP
        else:
            motor2.motor_speed = MOTOR2_SPEED_START if direction == 0 else -MOTOR2_SPEED_START
            motor2.motor_current = MOTOR2_CURRENT_START

    elif cmd == CMD_MOTOR1_TEST and len(payload) == 1:
        print("锁紧电机测试指令")
        direction = payload[0]
        motor1.motor_speed = MOTOR1_SPEED_START if direction == 0 else -MOTOR1_SPEED_START
        motor1.motor_current = MOTOR1_CURRENT_START
        motor1_impulse_counter = IMPULSE_DURATION

    elif cmd == CMD_MOTOR2_TEST and len(payload) == 1:
        print("充电电机测试指令")
        direction = payload[0]
        motor2.motor_speed = MOTOR2_SPEED_START if direction == 0 else -MOTOR2_SPEED_START
        motor2.motor_current = MOTOR2_CURRENT_START
        motor2_impulse_counter = IMPULSE_DURATION

    elif cmd == CMD_CHARGER_ONOFF and len(payload) == 1:
        print("无线充电控制指令")
        enable = payload[0]
        if enable == 0x01:
            charger.set_on_off = 0xB1  # 充电使能
        else:
            charger.set_on_off = 0xB2  # 停止充电

def upload_status():
    frame = build_frame()
    try:
        ser.write(frame)
    except serial.SerialException as e:
        print(f"Serial write failed: {e}")
    
    # 打印
    # # 英文标签（长度统一）
    # labels = [
    #     "distance1", "distance2", "roll", "pitch", "yaw",
    #     "flow_spd", "flow_dir", "servo", "weight", "switch",
    #     "servo_st", "m1_st", "m2_st"
    # ]

    # # 对应值
    # values = [
    #     displacement.displacement_distance,
    #     ultrasonic.uwc_distance,
    #     imu.imu_roll,
    #     imu.imu_pitch,
    #     imu.imu_yaw,
    #     flowmeter.flow_speed,
    #     flowmeter.flow_direction,
    #     servo.read_angle,
    #     load.weight,
    #     switch.switch_state,
    #     f"0x{servo.servo_state:02X}",
    #     f"0x{motor1.motor_status:02X}",
    #     f"0x{motor2.motor_status:02X}",
    # ]

    # # 标签行
    # print(' '.join(f"{label:>10}" for label in labels))
    # # 数值行
    # print(' '.join(
    #     f"{value:>10.3f}" if isinstance(value, float) else f"{value:>10}"
    #     for value in values
    # ))

events = {
    'f': Event(),  # displacement
    'e': Event(),  # ultrasonic, imu, flowmeter, load, switch
    'b': Event(),  # servo
    'c': Event(),  # motor1
    'd': Event(),  # motor2
    'h': Event(),  # charger
}

status_queues = {
    'f': Queue(),  # displacement
    'e': Queue(),  # ultrasonic, imu, flowmeter, load, switch
    'b': Queue(),  # servo
    'c': Queue(),  # motor1
    'd': Queue(),  # motor2
    'h': Queue(),  # charger
}

param_queues = {
    'servo': Queue(),    # (b)
    'motor1': Queue(),   # (c)
    'motor2': Queue(),   # (d)
    'charger': Queue(),  # (h)
}

def worker_b(event, param_queue, status_queue):
    while True:
        event.wait()

        # 读取
        result = servo.read_position()

        # 控制
        if not param_queue.empty():
            angle = param_queue.get()
            servo.set_angle = angle
            servo.set_position()
        else:
            print("[WARN] 控制参数未提供，跳过控制")

        # 传回主进程        
        status_queue.put({
            'data': result,
            'success': servo.read_success,
            'status': servo.servo_state
        })

        event.clear()

def worker_c(event, param_queue, status_queue):
    while True:
        event.wait()
        # 读取
        result = motor1.read_temperature()

        # 控制
        if not param_queue.empty():
            speed, current = param_queue.get()
            motor1.motor_speed = speed
            motor1.motor_current = current
            motor1.set_speed_and_current()
        else:
            print("[WARN] 控制参数未提供，跳过控制")        

        # 传回主进程
        status_queue.put({
            'data': result,
            'success': motor1.read_success, 
            'status': motor1.motor_status
        })

        event.clear()

def worker_d(event, param_queue, status_queue):
    while True:
        event.wait()

        # 读取
        result = motor2.read_temperature()

        # 控制
        if not param_queue.empty():
            speed, current = param_queue.get()
            motor2.motor_speed = speed
            motor2.motor_current = current
            motor2.set_speed_and_current()
        else:
            print("[WARN] 控制参数未提供，跳过控制")        
            
        # 传回主进程        
        status_queue.put({
            'data': result,
            'success': motor2.read_success,
            'status': motor2.motor_status
        })

        event.clear()

def worker_e(event, queue):
    while True:
        event.wait()
        result = {
            'ultra': {'data': ultrasonic.read_ultrasonic_distance(), 'success': ultrasonic.read_success},
            'imu': {'data': imu.read_imu_data(), 'success': imu.read_success},
            'flow': {'data': flowmeter.read_flowmeter(), 'success': flowmeter.read_success},
            'weight': {'data': load.read_weight_data(), 'success': load.read_success},
            'switch': {'data': switch.read_switch_state(), 'success': switch.read_success},
        }
        queue.put({'data': result})

        event.clear()
        
def worker_f(event, queue):
    while True:
        event.wait()
        result = displacement.read_displacement()
        queue.put({'data': result, 'success': displacement.read_success})

        event.clear()

def worker_h(event, param_queue, status_queue):
    while True:
        event.wait()

        # 读取
        result = charger.read_charger_status()

        # 控制
        if not param_queue.empty():
            value = param_queue.get()
            charger.set_on_off = value
            charger.set_charger()
        else:
            print("[WARN] charger 控制参数未提供，跳过控制")

        # 传回主进程
        status_queue.put({
            'data': result,
            'success': charger.read_success,
        })

        event.clear()


def start_workers():
    # displacement 路（f）
    Process(
        target=worker_f,
        args=(events['f'], status_queues['f']),
        daemon=True
    ).start()

    # ultrasonic/imu/flowmeter/load/switch 路（e）
    Process(
        target=worker_e,
        args=(events['e'], status_queues['e']),
        daemon=True
    ).start()

    # servo 路（b）：读取 + 控制
    Process(
        target=worker_b,
        args=(events['b'], param_queues['servo'], status_queues['b']),
        daemon=True
    ).start()

    # motor1 路（c）：读取 + 控制
    Process(
        target=worker_c,
        args=(events['c'], param_queues['motor1'], status_queues['c']),
        daemon=True
    ).start()

    # motor2 路（d）：读取 + 控制
    Process(
        target=worker_d,
        args=(events['d'], param_queues['motor2'], status_queues['d']),
        daemon=True
    ).start()

    # charger 路（h）：读取 + 控制
    Process(
        target=worker_h,
        args=(events['h'], param_queues['charger'], status_queues['h']),
        daemon=True
    ).start()

def read_and_ctrl():

    param_queues['servo'].put(servo.set_angle)
    param_queues['motor1'].put((motor1.motor_speed, motor1.motor_current))
    param_queues['motor2'].put((motor2.motor_speed, motor2.motor_current))
    param_queues['charger'].put(charger.set_on_off)

    for evt in events.values():
        evt.set()

    res_b = status_queues['b'].get()
    res_c = status_queues['c'].get()
    res_d = status_queues['d'].get()
    res_e = status_queues['e'].get()
    res_f = status_queues['f'].get()
    res_h = status_queues['h'].get()

    displacement.read_success = res_f['success']
    if displacement.read_success:
        displacement.displacement_distance = res_f['data']

    e = res_e['data']
    ultrasonic.read_success = e['ultra']['success']
    if ultrasonic.read_success:
        ultrasonic.uwc_distance, ultrasonic.uwc_status, ultrasonic.uwc_temperature = e['ultra']['data']
    imu.read_success = e['imu']['success']
    if imu.read_success:
        imu.imu_roll, imu.imu_pitch, imu.imu_yaw = e['imu']['data']
    flowmeter.read_success = e['flow']['success']
    if flowmeter.read_success:
        flowmeter.flow_direction, flowmeter.flow_speed = e['flow']['data']
    load.read_success = e['weight']['success']
    if load.read_success:
        load.weight = e['weight']['data']
    switch.read_success = e['switch']['success']
    if switch.read_success:
        switch.switch_state = e['switch']['data']

    servo.read_success = res_b['success']
    if servo.read_success:
        servo.read_angle = res_b['data']
        servo.servo_state = res_b.get('status', 0x00)

    motor1.read_success = res_c['success']
    if motor1.read_success:
        motor1.T_coil, motor1.T_pcb = res_c['data']
        motor1.motor_status = res_c.get('status', 0x00)

    motor2.read_success = res_d['success']
    if motor2.read_success:
        motor2.T_coil, motor2.T_pcb = res_d['data']
        motor2.motor_status = res_d.get('status', 0x00)
    
    charger.read_success = res_h['success']
    if charger.read_success:
        (charger.out_voltage,
         charger.out_current,
         charger.chg_state,
         charger.error_code,
         charger.input_volt,
         charger.input_current,
         charger.sender_temperature) = res_h['data'] 

    # print("read_success:",
    #     "disp", color_bool(displacement.read_success),
    #     "ultr", color_bool(ultrasonic.read_success),
    #     "imu ", color_bool(imu.read_success),
    #     "flow", color_bool(flowmeter.read_success),
    #     "load", color_bool(load.read_success),
    #     "swit", color_bool(switch.read_success),
    #     "sero", color_bool(servo.read_success),
    #     "mot1", color_bool(motor1.read_success),
    #     "mot2", color_bool(motor2.read_success),
    #     "chgr", color_bool(charger.read_success),
    # )

# 10Hz定时任务
T0 = 0
def periodic_task():
    global T0

    # real_period = time.perf_counter() - T0
    # T0 = time.perf_counter()
    # print(f"Period: {real_period :>10.5f}")    

    timestamps = []
    # timestamps.append(time.perf_counter())

    # 读取并解析指令
    process_received_data()
    # timestamps.append(time.perf_counter())

    # 电机保护逻辑
    motor1_motor2_protect()
    # timestamps.append(time.perf_counter())

    # 处理点动模式计数与停止动作
    handle_motor_impulses()
    # timestamps.append(time.perf_counter())

    # 采集控制所有传感器和执行器状态
    # read_all_devices()
    read_and_ctrl()
    # timestamps.append(time.perf_counter())

    # 构造帧并发送
    upload_status()
    # timestamps.append(time.perf_counter())

    # 控制伺服与电机
    # motor_ctrl()
    # timestamps.append(time.perf_counter())

    # 打印各阶段耗时
    # durations = [t1 - t0 for t0, t1 in zip(timestamps, timestamps[1:])]
    # print(" ".join(f"{d:>10.5f}" for d in durations))
    
    # 启动下一轮定时任务
    # threading.Timer(0.0001, periodic_task).start()
    # threading.Timer(PERIODIC_INTERVAL + PERIODIC_COMP, periodic_task).start()

def periodic_loop():
    while True:
        start = time.perf_counter()

        periodic_task()

        elapsed = time.perf_counter() - start
        time.sleep(max(0, PERIODIC_INTERVAL - elapsed))

if __name__ == '__main__':

    # 压力传感器清零
    load.read_weight_data()
    if load.weight < 0.05:
        load.reset_weight()

    # 伺服误差设置
    servo.set_error_value = 0.9
    servo.set_error()

    # 启动后保持伺服当前位置
    while True:
        servo.read_position()
        if servo.read_success:
            servo.set_angle = servo.read_angle
            break
        delay_precise(0.05)

    # 使能电机1 2
    motor1.motor_enable = 0x01
    motor1.enable_motor()
    motor2.motor_enable = 0x01
    motor2.enable_motor()

    # 启动子进程
    start_workers()
    # start_sensor_workers()
    # start_motor_workers()

    # 主任务循环
    threading.Thread(target=periodic_loop, daemon=True).start()
    while True:
        time.sleep(1)

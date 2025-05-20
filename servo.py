import serial
import time
import struct

# read_and_ctrl
# f| displacement_W displacement_R |
# e| ultrasonic_W ultrasonic_R imu_W imu_R flowmeter_W flowmeter_R load_W load_R |
# b| servo_W servo_R servo_W|
# c| motor1_W motor1_R motor1_W |
# d| motor2_W motor2_R motor2_W | 

# 全局变量
read_angle = 0  # 当前读取的角度
set_angle = 0   # 当前设置的角度（初始为0）
set_error_value = 0.9 # 当前误差参数（初始为0.9）
servo_state = 0 # 伺服电机的运动状态，0x01 正转，0x00 停转，0xF1 反转

# 方向滤波参数
servo_state_last = 0
servo_state_counter = 0
servo_state_threshold = 4  # 连续满足条件的次数

# 角度限制
set_angle_MAX = 35   # 设置最大角度
set_angle_MIN = -35  # 设置最小角度

read_success = False

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB1'
BAUD_RATE = 9600
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.04  # 等待时间 >=0.031
# TIME_WAIT_WONLY = 0.014 # 只写不读等待时间
READ_BYTES = 13

# 调试开关
DEBUG = False  # 设置为 True 来启用调试输出

def delay_precise(duration_s):
    start = time.perf_counter()
    end = start + duration_s
    while True:
        now = time.perf_counter()
        if now >= end:
            break

# 设置串口
def setup_serial():
    # ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, parity=PARITY, timeout=0)
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, parity=PARITY, timeout=TIME_WAIT)
    return ser

# 校验和计算 不是modbus
def calculate_checksum(data):
    # 计算从第 1 到第 8 字节的校验和
    checksum = sum(data[1:9]) & 0xFFFF
    return checksum & 0xFFFF

# 校验和验证 不是modbus
def verify_checksum(response):
    checksum = calculate_checksum(response)
    # 校验和是倒数第三和倒数第二个字节
    response_checksum = struct.unpack('>H', response[-4:-2])[0]
    if DEBUG:
        print(f"Calculated checksum: {checksum:04X}")
        print(f"Response checksum: {response_checksum:04X}")
    return checksum == response_checksum

def write_all(ser, data_to_send):
    while True:
        written = ser.write(data_to_send)
        if written == len(data_to_send):
            break  # 完全写入成功，退出循环

# 发送指令到伺服电机
def send_command(data_to_send, read_bytes=0):
    ser = setup_serial()
    write_all(ser, data_to_send)
    if DEBUG:
        print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")
    
    if read_bytes > 0:
        # delay_precise(TIME_WAIT)
        response = ser.read(read_bytes)
        if DEBUG:
            print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        # delay_precise(TIME_WAIT_WONLY)
        return None

# 读取位置，带符号位处理
def read_position():
    global read_angle, servo_state, read_success
    data_to_send = bytearray([0x3A, 0x81, 0x01, 0x00, 0x06, 0x01, 0x01, 0x00, 0x8A, 0x0D, 0x0A])
    response = send_command(data_to_send, READ_BYTES)
    
    if len(response) == READ_BYTES:
        # 校验和验证
        if not verify_checksum(response):
            if DEBUG:
                print("校验和错误")
            read_success = False
            return None
        
        # 读取位置的字节
        angle_bytes = response[7:9]  # 这里读取符号位和数据
        if DEBUG:
            print(f"angle_bytes:    {' '.join(f'{x:02X}' for x in angle_bytes)}")
        
        # 提取符号位和数据
        sign_bit = angle_bytes[0] & 0x80  # 获取符号位（最高位）
        position = (angle_bytes[0] & 0x7F) << 8 | angle_bytes[1]  # 获取角度的绝对值
        
        # 如果符号位为1，则角度为负数
        new_read_angle = -position / 10 if sign_bit else position / 10  # 转换为角度
        
        # 推断当前方向
        if new_read_angle > read_angle:
            new_state = 0x01  # 正转
        elif new_read_angle == read_angle:
            new_state = 0x00  # 停转
        else:
            new_state = 0xF1  # 反转

        # 滤波判断
        global servo_state_last, servo_state_counter, servo_state
        if new_state == servo_state_last:
            servo_state_counter += 1
        else:
            servo_state_last = new_state
            servo_state_counter = 1

        if servo_state_counter >= servo_state_threshold:
            servo_state = new_state  # 满足条件后才真正更新状态
        
        # 更新角度
        read_angle = new_read_angle
        
        read_success = True
        return read_angle
    else:
        read_success = False
        return None

# 设置位置，带保护值和负角度处理
def set_position():
    global set_angle
    # 限制角度范围
    if set_angle > set_angle_MAX:
        set_angle = set_angle_MAX
        if DEBUG:
            print(f"Angle exceeds max limit. Setting to {set_angle_MAX}°.")
    elif set_angle < set_angle_MIN:
        set_angle = set_angle_MIN
        if DEBUG:
            print(f"Angle exceeds min limit. Setting to {set_angle_MIN}°.")
        
    # 对于负角度，符号位处理为0x80
    position = abs(int(set_angle * 10))  # 角度转为整数，乘以10，取绝对值
    if set_angle < 0:
        # 设置符号位为0x80表示负数
        data_to_send = bytearray([0x3A, 0x81, 0x01, 0x00, 0x02, 0x03, 0x01, (position >> 8) | 0x80, position & 0xFF])
    else:
        # 正角度，直接传递位置值
        data_to_send = bytearray([0x3A, 0x81, 0x01, 0x00, 0x02, 0x03, 0x01, (position >> 8) & 0xFF, position & 0xFF])

    # 计算校验和
    checksum = calculate_checksum(data_to_send)
    checksum_h = (checksum >> 8) & 0xFF  # 高字节
    checksum_l = checksum & 0xFF         # 低字节

    # 完整的指令，包括校验和
    data_to_send.extend([checksum_h, checksum_l, 0x0D, 0x0A])
    send_command(data_to_send)

# 设置误差
def set_error():
    global set_error_value
    data_to_send = bytearray([0x3A, 0x01, 0x81, 0x00, 0x11, 0x02, 0x01, int(set_error_value * 10), 0x00, 0x9F, 0x0D, 0x0A])
    send_command(data_to_send)

# 停机保持
def stop_motor():
    data_to_send = bytearray([0x3A, 0x81, 0x01, 0x00, 0x01, 0x03, 0x01, 0xE8, 0x01, 0x6F, 0x0D, 0x0A])
    send_command(data_to_send)

def color_bool(value: bool) -> str:
    text = f"{value:>5}"  # 固定宽度为5，右对齐
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 测试函数
def test_servo():
    global set_angle, set_error_value
    test_angle = -40  # 30°
    test_angle1 = 0
    test_wait = 30  # 30s
    test_error = 0.5  # 0.5°

    print("测试:")

    # print("")
    # print("设置误差")
    # print(f"set_error_value = {test_error}")
    # set_error_value = test_error
    # set_error()
    
    # print("")
    # print("读取角度")
    # angle = read_position()
    # if angle is not None:
    #     print(f"{angle} = read_position()")
    # else:
    #     print("读取失败")
    
    # print("")
    # print("设置位置")
    # print(f"set_position({test_angle})")
    # set_angle = test_angle
    # set_position()
    
    # print("")
    # print(f"time.sleep({test_wait})")
    # time.sleep(test_wait)
    
    # print("")
    # print(f"读取角度")
    # angle = read_position()
    # print(f"{angle} = read_position()")
    
    # # 输出 servo_state，以16进制格式显示，并标明运动状态
    # print(f"servo_state = {servo_state:02X} ({'正转' if servo_state == 0x01 else '停转' if servo_state == 0x00 else '反转'})")

    # print("")
    # print("设置位置")
    # print(f"set_position({test_angle1})")
    # set_angle = 0
    # set_position()

    # print("")
    # print("time.sleep(5)")
    # time.sleep(5)
    
    # print("")
    # print("停机保持")
    # print(f"stop_motor()")
    # stop_motor()

    # print("time.sleep(5)")
    # time.sleep(5)

    # print("")
    # print(f"读取角度")
    # angle = read_position()
    # print(f"{angle} = read_position()")
    # # 输出 servo_state
    # print(f"servo_state = {servo_state:02X} ({'正转' if servo_state == 0x01 else '停转' if servo_state == 0x00 else '反转'})")

    # print("time.sleep(5)")
    # time.sleep(5)

    # print("")
    # print(f"读取角度")
    # angle = read_position()
    # print(f"{angle} = read_position()")
    # # 输出 servo_state
    # print(f"servo_state = {servo_state:02X} ({'正转' if servo_state == 0x01 else '停转' if servo_state == 0x00 else '反转'})")

    # print("time.sleep(5)")
    # time.sleep(5)

    # print("")
    # print(f"读取角度")
    # angle = read_position()
    # print(f"{angle} = read_position()")
    # # 输出 servo_state
    # print(f"servo_state = {servo_state:02X} ({'正转' if servo_state == 0x01 else '停转' if servo_state == 0x00 else '反转'})")

    # print("")
    # print("设置位置")
    # print(f"set_position({test_angle1})")
    # set_angle = test_angle1
    # set_position()

    # print("")
    # print("设置误差")
    # print(f"set_error_value = {test_error}")
    # set_error()
    # while True:        
    #     print("")
    #     print("读取角度")
    #     angle = read_position()
    #     if angle is not None:
    #         print(f"{angle} = read_position()")
    #     else:
    #         print("读取失败")
    #     delay_precise(0.1)

    while True:
        read_position()
        set_position()
        print(color_bool(read_success))

if __name__ == '__main__':
    test_servo()

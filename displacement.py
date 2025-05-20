import serial
import time
import struct

# 全局变量

# 当前读取的位移值（单位：cm）
displacement_distance = 0.0

read_success = False

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB5'
BAUD_RATE = 9600
PARITY = serial.PARITY_EVEN
TIME_WAIT = 0.03  # 等待时间 >=0.024
READ_BYTES = 9

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

# CRC16 校验计算（Modbus 标准）
def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def write_all(ser, data_to_send):
    while True:
        written = ser.write(data_to_send)
        if written == len(data_to_send):
            break  # 完全写入成功，退出循环

# 通用发送函数
def send_command(data_to_send, read_bytes=0):
    ser = setup_serial()
    write_all(ser, data_to_send)

    # delay_precise(TIME_WAIT)
    if DEBUG:
        print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")

    if read_bytes > 0:
        response = ser.read(read_bytes)
        if DEBUG:
            print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        return None

# 读取位移值
def read_displacement():
    global displacement_distance, read_success
    data_to_send = bytearray([0x18, 0x03, 0x00, 0x04, 0x00, 0x02, 0x87, 0xC3])
    response = send_command(data_to_send, READ_BYTES)

    if len(response) == READ_BYTES:
        # 校验 CRC
        received_crc = struct.unpack('<H', response[7:9])[0]
        calculated_crc = calculate_crc(response[:7])

        if DEBUG:
            print(f"Calculated CRC: {calculated_crc:04X}")
            print(f"Received CRC: {received_crc:04X}")

        if received_crc == calculated_crc:
            integer_value, dot_value = struct.unpack('>HH', response[3:7])
            displacement_distance = integer_value / 10.0 + dot_value / 65535.0
            read_success = True
            return displacement_distance
        else:
            if DEBUG:
                print("CRC 校验失败")
            read_success = False
            return None
    else:
        if DEBUG:
            print("无响应或响应长度错误")
        read_success = False
        return None

def color_bool(value: bool) -> str:
    text = f"{value:>5}"  # 固定宽度为5，右对齐
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 测试函数
def test_displacement():
    #  while True:
    #     print("读取位移值:")
    #     distance = read_displacement()
    #     if distance is not None:
    #         print(f"displacement_distance = {displacement_distance:.4f} cm")
    #     else:
    #         print("读取失败")
    #     delay_precise(0.5)

     while True:
        read_displacement()
    # 打印状态
        print(color_bool(read_success))

if __name__ == '__main__':
    test_displacement()

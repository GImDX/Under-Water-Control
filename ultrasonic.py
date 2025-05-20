import serial
import time
import struct

# 全局变量
uwc_distance = 0.0  # 当前超声波测距（单位：cm）
uwc_status = 0 # 当前状态 是否入水 0/1
uwc_temperature = 0 # 当前温度 摄氏度

read_success = False

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB4'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.020  # 等待时间 >=0.016
READ_BYTES = 11

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

# 读取超声波测距值
def read_ultrasonic_distance():
    global uwc_distance, uwc_status, uwc_temperature, read_success
    data_to_send = bytearray([0x03, 0x03, 0x00, 0x10, 0x00, 0x03, 0x05, 0xEC])
    response = send_command(data_to_send, read_bytes=READ_BYTES)

    if len(response) == READ_BYTES:
        # CRC 校验
        crc_received = struct.unpack('<H', response[9:11])[0]
        crc_calculated = calculate_crc(response[:9])

        if DEBUG:
            print(f"Calculated CRC: {crc_calculated:04X}")
            print(f"Received CRC: {crc_received:04X}")

        if crc_received == crc_calculated:
            # 解析距离值
            distance_raw = struct.unpack('>H', response[3:5])[0]
            uwc_distance = distance_raw / 100.0  # 转换为厘米
            uwc_status = struct.unpack('>H', response[5:7])[0]
            uwc_temperature = struct.unpack('>H', response[7:9])[0]
            read_success = True
            return uwc_distance, uwc_status, uwc_temperature
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
def test_ultrasonic():
    # while True:
    #     print("读取超声波测距值:")
    #     ultrasonic_data = read_ultrasonic_distance()
    #     if ultrasonic_data is not None:
    #         uwc_distance, uwc_status, uwc_temperature = ultrasonic_data
    #         print(f"uwc_distance = {uwc_distance:.4f} cm    uwc_status = 0x{uwc_status & 0xFF:02X}  uwc_temperature = {uwc_temperature:d} ℃")
    #     else:
    #         print("读取失败")
    #     delay_precise(0.05)

    while True:
        read_ultrasonic_distance()
        print(color_bool(read_success))


    # 修改从站地址为 0x03
    # print("修改从站地址为 0x02")
    # data_to_send = bytearray([0x01, 0x06, 0x00, 0x15, 0x00, 0x03, 0xD8, 0x0F])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

if __name__ == '__main__':
    test_ultrasonic()

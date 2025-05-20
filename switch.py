import serial
import struct
import time

# 定义全局变量
switch_state = 0  # 用于储存开关状态；0x00=未触发，0x01=触发

read_success = False

# 定义串口设置
SERIAL_PORT = '/dev/ttyCH9344USB4'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.01  # 等待时间 >=0.008
# TIME_WAIT_W = 0.005  # 写入前等待时间
READ_BYTES = 7

# DEBUG 开关
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

# CRC16 校验计算
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
    # delay_precise(TIME_WAIT_W)
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

# 读取接近开关状态
def read_switch_state():
    global switch_state, read_success
    data_to_send = bytearray([0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x31, 0xF9])
    response = send_command(data_to_send, READ_BYTES)

    if len(response) == READ_BYTES:
        calculated_crc = calculate_crc(response[:-2])
        received_crc = struct.unpack('<H', response[-2:])[0]
        
        if DEBUG:
            print(f"Calculated CRC: {calculated_crc:04X}")
            print(f"Received CRC: {received_crc:04X}")

        if calculated_crc == received_crc:
            switch_state = response[4]
            if DEBUG:
                print(f"Switch state: {switch_state:02X}")
            read_success = True
            return switch_state
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
def test_switch():
    # while True:
    #     result = read_switch_state()
    #     if result is not None:
    #         status_str = "触发" if switch_state == 0x01 else "未触发"
    #         print(f"开关状态: {switch_state:02X} ({status_str})")
    #     else:
    #         print("读取失败")
    #     time.sleep(0.05)

    while True:
        read_switch_state()
        print(color_bool(read_success))

    # 修改波特率为 115200
    # print("修改波特率为 115200")
    # data_to_send = bytearray([0x01, 0x06, 0x00, 0x33, 0x00, 0x07, 0x38, 0x07])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 修改从站地址为 0x02
    # print("修改从站地址为 0x02")
    # data_to_send = bytearray([0x01, 0x06, 0x00, 0x32, 0x00, 0x02, 0xA9, 0xC4])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

if __name__ == '__main__':
    test_switch()

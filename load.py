import serial
import time
import struct

# 全局变量
weight = 0.0  # 当前重量值

read_success = False

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB4'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.01  # 等待时间 >=0.004
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
    ser = setup_serial()
    write_all(ser, data_to_send)
    # delay_precise(TIME_WAIT)
    if DEBUG:
        print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")

    if read_bytes > 0:
        response = ser.read(read_bytes)
        # 必须sleep一段时间等待load释放总线 >=0.005
        time.sleep(0.01)
        if DEBUG:
            print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        return None

# 读取重量数据
def read_weight_data():
    global weight, read_success
    data_to_send = bytearray([0x01, 0x03, 0x00, 0x50, 0x00, 0x02, 0xC4, 0x1A])
    response = send_command(data_to_send, READ_BYTES)
    
    if len(response) == READ_BYTES:
        # 验证CRC校验
        data_without_crc = response[:-2]  # CRC之前的数据
        crc_received = struct.unpack('<H', response[-2:])[0]  # 接收到的CRC值
        crc_calculated = calculate_crc(data_without_crc)  # 计算的CRC值
        if DEBUG:
            print(f"Calculated CRC: {crc_calculated:04X}")
            print(f"Received CRC: {crc_received:04X}")

        if crc_received != crc_calculated:
            if DEBUG:
                print(f"CRC错误! 计算值: {crc_calculated:04X}, 接收值: {crc_received:04X}")
            read_success = False
            return None
        
        # 解析重量数据
        weight_data = struct.unpack('>i', bytes(response[3:7]))[0]  # 转换为32位有符号整型 大端模式
        weight = weight_data / 100.0  # 转换为kg
        if DEBUG:
            print(f"Weight: {weight:.2f} kg")
        read_success = True
        return weight
    else:
        read_success = False
        return None

# 清零传感器
def reset_weight():
    data_to_send = bytearray([0x01, 0x10, 0x00, 0x5E, 0x00, 0x01, 0x02, 0x00, 0x01, 0x6A, 0xEE])
    response = send_command(data_to_send, 8)
    if DEBUG:
        print("Clearing weight sensor...")

def color_bool(value: bool) -> str:
    text = f"{value:>5}"  # 固定宽度为5，右对齐
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 测试函数
def test_weight():
    # read_weight_data()
    # if weight < 0.05:
    #     print("清零传感器")
    #     reset_weight()
    # print("测试重量读取:")
    # while True:
    #     weight_data = read_weight_data()
    #     if weight_data is not None:
    #         print(f"Weight: {weight_data:.2f} kg")
    #     else:
    #         print("读取失败")
    #     delay_precise(0.1)

    while True:
        read_weight_data()
        print(color_bool(read_success))


    # 读重量单位 response[03] = 0x02为kg
    # data_to_send = bytearray([0x01, 0x03, 0x00, 0x68, 0x00, 0x02, 0x45, 0xD7])
    # response = send_command(data_to_send, 6)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 零点标定
    # print("零点标定 空置传感器")
    # data_to_send = bytearray([0x01, 0x10, 0x00, 0x26, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x71, 0x9D])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 增益标定 5.1kg砝码
    # print("增益标定 放置5.1kg砝码")
    # data_to_send = bytearray([0x01, 0x10, 0x00, 0x2A, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01, 0xFE, 0xF1, 0xD8])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # # 修改波特率
    # print("修改波特率至115200")
    # # 解锁，允许修改波特率
    # data_to_send = bytearray([0x01, 0x10, 0x00, 0x05, 0x00, 0x01, 0x02, 0x5A, 0xA5, 0x5C, 0xDE])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
    # # 波特率设置
    # data_to_send = bytearray([0x01, 0x10, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x07, 0xE6, 0x43])
    # response = send_command(data_to_send, 1)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # delay_precise(2)
    # weight_data = read_weight_data()
    # print(f"Weight: {weight_data:.2f} kg")

if __name__ == '__main__':
    test_weight()

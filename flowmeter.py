import serial
import time
import struct

# 全局变量
flow_direction = 0
flow_speed = 0

read_success = False

# 配置串口参数
SERIAL_PORT = '/dev/ttyCH9344USB4'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.01  # 等待时间 >=0.004
READ_BYTES = 10

# DEBUG模式开关
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
        # delay_precise(0.003)
        # 必须sleep一段时间等待flowmeter释放总线 >=0.002
        time.sleep(0.005)
        if DEBUG:
            print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        return None

def read_flowmeter():
    global flow_direction, flow_speed, read_success
    data_to_send = bytearray([0x05, 0x20, 0x00, 0x04, 0x00, 0x00, 0xC0, 0x48])
    response = send_command(data_to_send, READ_BYTES)

    if len(response) == READ_BYTES:
        # 校验 CRC
        data_without_crc = response[:8]  # CRC之前的数据
        received_crc = struct.unpack('<H', response[8:10])[0]  # 接收到的CRC值
        calculated_crc = calculate_crc(data_without_crc)  # 计算的CRC值

        if DEBUG:
            print(f"Calculated CRC: {calculated_crc:04X}")
            print(f"Received CRC: {received_crc:04X}")

        if received_crc == calculated_crc:
            # 提取流向数据
            flow_direction_raw = struct.unpack('<H', response[4:6])[0]  # 反转字节序
            flow_direction = flow_direction_raw / 10.0  # 转换为角度（度）
            if DEBUG:
                print(f"flow_direction_raw: {flow_direction_raw:04X}")

            # 提取流速数据
            flow_speed_raw = struct.unpack('<h', response[6:8])[0]  # 反转字节序
            flow_speed = flow_speed_raw / 100.0  # 转换为流速（m/s）
            if DEBUG:
                print(f"flow_speed_raw: {flow_speed_raw:04X}")
            read_success = True
            return flow_direction, flow_speed
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
def test_flowmeter():
    # while True:
    #     # 读取流速流向仪数据
    #     read_flowmeter()
    #     print(f"流向: {flow_direction}° 流速: {flow_speed} m/s")
    #     time.sleep(0.5)

    while True:
        read_flowmeter()
        print(color_bool(read_success))

if __name__ == "__main__":
    test_flowmeter()

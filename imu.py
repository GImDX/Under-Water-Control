import serial
import time
import struct

# 全局变量
imu_roll = 0  # IMU的滚转角度
imu_pitch = 0  # IMU的俯仰角度
imu_yaw = 0  # IMU的偏航角度

read_success = False

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB4'
BAUD_RATE = 115200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.005  # 等待时间 >=0.002
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
        if DEBUG:
            print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        return None

# 读取IMU角度数据
def read_imu_data():
    global imu_roll, imu_pitch, imu_yaw, read_success
    data_to_send = bytearray([0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x99, 0x86])
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
        
        # 解析角度数据
        imu_roll_raw, imu_pitch_raw, imu_yaw_raw = struct.unpack('>hhh', response[3:9])
        imu_roll  = imu_roll_raw  / 32768 * 180
        imu_pitch = imu_pitch_raw / 32768 * 180
        imu_yaw   = imu_yaw_raw   / 32768 * 180

        if DEBUG:
            print(f"Roll: {imu_roll:.2f}°")
            print(f"Pitch: {imu_pitch:.2f}°")
            print(f"Yaw: {imu_yaw:.2f}°")
        
        read_success = True
        return imu_roll, imu_pitch, imu_yaw
    else:
        read_success = False
        return None
    
def color_bool(value: bool) -> str:
    text = f"{value:>5}"  # 固定宽度为5，右对齐
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 测试函数
def test_imu():
    # print("测试IMU角度读取:")
    # while True:
    #     imu_data = read_imu_data()
    #     if imu_data:
    #         imu_roll, imu_pitch, imu_yaw = imu_data
    #         print(f"IMU 数据 -> Roll: {imu_roll:.2f}°  Pitch: {imu_pitch:.2f}°  Yaw: {imu_yaw:.2f}°")
    #     else:
    #         print("读取失败")
    #     delay_precise(0.01)

    while True:
        read_imu_data()
        print(color_bool(read_success))

    # 修改波特率至115200
    # print("修改波特率至115200")
    # 解锁，允许修改波特率
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
    # # 波特率设置
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x04, 0x00, 0x06, 0x45, 0x88])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
    # 保存 50 06 00 00 00 00 84 4B
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # # 读取设备编号
    # data_to_send = bytearray([0x50, 0x03, 0x00, 0x7F, 0x00, 0x06, 0xF9, 0x91])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 17)
    # # 选择指定字节顺序
    # indices = [4, 3, 6, 5, 8, 7, 10, 9, 12, 11]
    # selected_bytes = bytearray(response[i] for i in indices)
    # # 转换为 ASCII 字符串
    # ascii_string = selected_bytes.decode('ascii', errors='replace')  # 替换非法字符
    # print("NUMBERID:", ascii_string)

    # 读取延时
    # data_to_send = bytearray([0x50, 0x03, 0x00, 0x74, 0x00, 0x01, 0xC9, 0x91])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 写入延时 50 06 00 74 00 00 C4 51(0 us)
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x74, 0x00, 0x00, 0xC4, 0x51])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 读取角度算法 50 03 00 24 00 01 C9 80
    # 设置6轴 50 06 00 24 00 01 05 80
    # 设置9轴 50 06 00 24 00 00 C4 40
    # data_to_send = bytearray([0x50, 0x03, 0x00, 0x24, 0x00, 0x01, 0xC9, 0x80])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x24, 0x00, 0x01, 0x05, 0x80])
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x24, 0x00, 0x00, 0xC4, 0x40])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 读安装方向 50 03 00 23 00 01 78 41
    # data_to_send = bytearray([0x50, 0x03, 0x00, 0x23, 0x00, 0x01, 0x78, 0x41])
    # global TIME_WAIT
    # TIME_WAIT = 0.1
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")

    # 航向角置零 50 06 00 01 00 04 D4 48
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD4, 0x48])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
    # 保存 50 06 00 00 00 00 84 4B
    # data_to_send = bytearray([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])
    # response = send_command(data_to_send, 8)
    # print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")


if __name__ == '__main__':
    test_imu()

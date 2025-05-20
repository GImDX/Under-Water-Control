import serial
import time
import struct

# 无线充电模块ID
CHG_ID = 0x01

# 串口设置
SERIAL_PORT = '/dev/ttyCH9344USB7'
BAUD_RATE = 19200
PARITY = serial.PARITY_NONE
TIME_WAIT = 0.05
READ_BYTES = 35  # 读取15个寄存器，每个2字节，加上5字节头尾

# 全局变量
out_voltage = 0.0          # 输出电压
out_current = 0.0          # 输出电流
chg_state = 0              # 充电状态
error_code = 0             # 故障码
input_volt = 0.0           # 输入电压
input_current = 0.0        # 输入电流
sender_temperature = 0.0   # 发射端温度

set_on_off = 0xB2          # 设置充电状态 0xB1开 0xB2关
read_success = False       # 读取成功标志

# 调试开关
DEBUG = False  # 设置为 True 来启用调试输出

def delay_precise(duration_s):
    start = time.perf_counter()
    while time.perf_counter() < start + duration_s:
        pass

def setup_serial():
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, parity=PARITY, timeout=TIME_WAIT)

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
            break

def send_command(data_to_send, read_bytes=0):
    ser = setup_serial()
    crc = calculate_crc(data_to_send)
    data_to_send.extend(struct.pack('<H', crc))
    write_all(ser, data_to_send)

    if DEBUG:
        print(f"Sent: {' '.join(f'{x:02X}' for x in data_to_send)}")

    if read_bytes > 0:
        response = ser.read(read_bytes)
        if DEBUG:
            print(f"Rcve: {' '.join(f'{x:02X}' for x in response)}")
        ser.close()
        return response
    else:
        ser.close()
        return None

# 设置充电开关（使用全局变量 set_on_off）
def set_charger():
    data_to_send = bytearray([CHG_ID, 0x06, 0x00, 0x00, 0x00, set_on_off])
    send_command(data_to_send, read_bytes=0)

# 查询输出状态
def read_charger_status():
    global out_voltage, out_current, chg_state, error_code
    global input_volt, input_current, sender_temperature, read_success

    # 读取0x02-0x10共15个寄存器，每个寄存器2字节，加上5字节Modbus头尾，总共读取35字节
    data_to_send = bytearray([CHG_ID, 0x03, 0x00, 0x02, 0x00, 0x0F])
    response = send_command(data_to_send, READ_BYTES)

    if len(response) != READ_BYTES:
        read_success = False
        return None

    crc_received = struct.unpack('<H', response[-2:])[0]
    crc_calculated = calculate_crc(response[:-2])
    if crc_received != crc_calculated:
        if DEBUG:
            print(f"CRC错误! 计算: {crc_calculated:04X}, 接收: {crc_received:04X}")
        read_success = False
        return None
    
    # 使用前 6 字节做 float 电压电流（H），然后第7-8 和 9-10字节为 signed short（h）
    values = struct.unpack('>2H2h11H', response[3:-2])  # 15个寄存器，每个2字节

    out_voltage = values[0] / 100.0
    out_current = values[1] / 100.0
    chg_state = values[2]
    error_code = values[3]
    input_volt = values[12] / 100.0
    input_current = values[13] / 100.0
    sender_temperature = values[14] / 100.0

    if DEBUG:
        print(f"out_voltage = {out_voltage:.2f} V")
        print(f"out_current = {out_current:.2f} A")
        print(f"chg_state = 0x{chg_state:04X}")
        print(f"error_code = 0x{error_code:04X}")
        print(f"input_volt = {input_volt:.2f} V")
        print(f"input_current = {input_current:.2f} A")
        print(f"sender_temperature = {sender_temperature:.1f} ℃")

    read_success = True
    return (
        out_voltage, out_current, chg_state, error_code,
        input_volt, input_current, sender_temperature
    )

def color_bool(value: bool) -> str:
    text = f"{value:>5}"
    return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

# 测试函数
def test_charger():
    while True:
        read_charger_status()
        print(color_bool(read_success))
        time.sleep(0.05)

if __name__ == '__main__':
    test_charger()

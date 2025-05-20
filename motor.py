# motor.py
import serial
import time
import struct
import math

class MotorController:
    # 常量值
    PI = math.pi
    MAX_SPEED_RPM = 20 * 180 / PI  # 20 rad/s 转换为 °/s
    TIME_WAIT_R = 0.051  # 读等待时间 >=0.051
    TIME_WAIT_W = 0.051  # 写等待时间 >=0.048

    def __init__(self, motor_id, serial_port):
        # 电机ID和串口
        self.motor_id = motor_id
        self.serial_port = serial_port

        # 电机运动状态
        self.motor_speed = 0.0  # 电机设置的转速
        self.motor_status = 0x00  # 电机运动状态（正转/停转/反转）
        self.motor_current = 0.0  # 电流值
        self.motor_enable = 0x00  # 电机使能状态
        self.motor_over_temp = 0x00 # 电机过温保护

        # 电机温度
        self.T_coil = 0.0  # 线圆温度
        self.T_pcb = 0.0   # 驱动板温度

        # 电机状态
        self.err_state = 0.0

        self.read_success = False  # 读取是否成功
        self.DEBUG = False  # 调试模式

    # 设置串口
    def setup_serial(self):
        # return serial.Serial(self.serial_port, baudrate=115200, parity=serial.PARITY_NONE, timeout=0)
        return serial.Serial(self.serial_port, baudrate=115200, parity=serial.PARITY_NONE, timeout=self.TIME_WAIT_R)

    # CRC16 校验计算
    def calculate_crc(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def write_all(self, ser, data_to_send):
        while True:
            written = ser.write(data_to_send)
            if written == len(data_to_send):
                break  # 完全写入成功，退出循环
    
    # | T0 motor_W T1+T2 motor_R T3+T0 motor_W T1 |
    # T0+T1=0.048 T1+T2=0.051 T3+T0 = 0.018
    # 发送 Modbus 指令并接收响应
    def send_command(self, data_to_send, read_bytes=0):
        # T0 = 0.021
        # T1 = 0.030
        # T2 = 0.021
        # T3 = 0.030
        # self.delay_precise(T0)
        ser = self.setup_serial()
        data_to_send.extend(struct.pack('<H', self.calculate_crc(data_to_send)))
        self.write_all(ser, data_to_send)
        # self.delay_precise(T1)

        if self.DEBUG:
            print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")

        if read_bytes > 0:
            # self.delay_precise(T2)
            response = ser.read(read_bytes)
            # self.delay_precise(T3)
            if self.DEBUG:
                print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
            ser.close()
            return response
        else:
            # 电机必须在发送控制指令后等待一段时间，否则读取无响应
            time.sleep(self.TIME_WAIT_W)
            ser.close()
            return None

    # 使能电机
    def enable_motor(self):
        data_to_send = bytearray([self.motor_id, 0x65, self.motor_enable])
        if self.DEBUG:
            print("Enabling motor")
        self.send_command(data_to_send)

    # 设置转速和电流
    def set_speed_and_current(self):
        if self.motor_speed > self.MAX_SPEED_RPM:
            self.motor_speed = self.MAX_SPEED_RPM
            print(f"Speed exceeds max limit. Setting to {self.MAX_SPEED_RPM}°.")
        elif self.motor_speed < -self.MAX_SPEED_RPM:
            self.motor_speed = -self.MAX_SPEED_RPM
            print(f"Speed exceeds min limit. Setting to {-self.MAX_SPEED_RPM}°.")

        # 转速(°/s)转换为 rad/s
        speed_rad_s = self.motor_speed * self.PI / 180

        # 转速(弧/s)和电流(A)转换为 IEEE754 浮点数格式
        speed_bytes = struct.pack('<f', speed_rad_s)
        current_bytes = struct.pack('<f', self.motor_current)

        # 更新电机运动状态
        if self.motor_speed > 0:
            self.motor_status = 0x01  # 正转
        elif self.motor_speed < 0:
            self.motor_status = 0xF1  # 反转
        else:
            self.motor_status = 0x00  # 停转

        data_to_send = bytearray([self.motor_id, 0x65, 0x06, *speed_bytes, *current_bytes])
        if self.DEBUG:
            print(f"Setting speed: {self.motor_speed}°/s, current: {self.motor_current} A")
            print(f"motor_status: {'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")

        self.send_command(data_to_send)

    # 读取电机温度
    def read_temperature(self):
        data_to_send = bytearray([self.motor_id, 0x04, 0x00, 0x0D, 0x00, 0x02])
        response = self.send_command(data_to_send, 13)
        if response and len(response) == 13:
            crc_received = struct.unpack('<H', response[-2:])[0]
            crc_calculated = self.calculate_crc(response[:-2])
            if crc_received == crc_calculated:
                self.T_coil = struct.unpack('<f', response[3:7])[0]
                self.T_pcb = struct.unpack('<f', response[7:11])[0]
                self.read_success = True
                return self.T_coil, self.T_pcb
        self.read_success = False
        return None
    
    # 读取电机温度
    def read_error(self):
        data_to_send = bytearray([self.motor_id, 0x04, 0x00, 0x0F, 0x00, 0x01])
        response = self.send_command(data_to_send, 9)
        if response and len(response) == 9:
            crc_received = struct.unpack('<H', response[-2:])[0]
            crc_calculated = self.calculate_crc(response[:-2])
            if crc_received == crc_calculated:
                self.err_state = struct.unpack('<f', response[3:7])[0]
                self.read_success = True
                return self.T_coil, self.T_pcb
        self.read_success = False
        return None
    
    # 清除错误
    def clear_error(self):
        data_to_send = bytearray([self.motor_id, 0x65, 0x02])
        if self.DEBUG:
            print("Clearing motor error")
        self.send_command(data_to_send)


    # 精确延时
    def delay_precise(self, duration_s):
        start = time.perf_counter()
        while time.perf_counter() < start + duration_s:
            pass

    def color_bool(self,value: bool) -> str:
        text = f"{value:>5}"  # 固定宽度为5，右对齐
        return f"\033[92m{text}\033[0m" if value else f"\033[91m{text}\033[0m"

    # 测试函数
    def test_motor(self):
        # print("正转/下降点动")
        # self.motor_enable = 0x01
        # self.enable_motor()
        # # self.delay_precise(0.05)  # 等待电机启动
        # self.motor_speed = 5.0 / self.PI * 180
        # self.motor_current = 0.7
        # self.set_speed_and_current()
        # print(f"电机当前转速：{self.motor_speed}°/s")
        # print(f"电机当前状态：{'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")
        # self.delay_precise(1)
        # print("self.delay_precise(1)")
        # print("设置转速为 0 rad/s, 电流为 0.0 A")
        # self.motor_speed = 0.0
        # self.motor_current = 0.0
        # self.set_speed_and_current()

        # print("反转/上升点动")
        # self.motor_enable = 0x01
        # self.enable_motor()
        # # self.delay_precise(0.05)
        # self.motor_speed = -5.0 / self.PI * 180
        # self.motor_current = 0.7
        # self.set_speed_and_current()
        # print(f"电机当前转速：{self.motor_speed}°/s")
        # print(f"电机当前状态：{'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")
        # print("self.delay_precise(1)")
        # self.delay_precise(1)
        # print("设置转速为 0 rad/s, 电流为 0.0 A")
        # self.motor_speed = 0.0
        # self.motor_current = 0.0
        # self.set_speed_and_current()

        # 读取温度
        # self.motor_enable = 0x00
        # self.enable_motor()
        # self.motor_enable = 0x01
        # self.enable_motor()

        # self.read_temperature()
        # print("T_coil: ", self.T_coil)
        # print("T_pcb: ", self.T_pcb)
        # self.clear_error()
        # print("self.clear_error()")
        # self.clear_error()
        # self.enable_motor()
        while True:
            # self.delay_precise(0.01)
            # self.enable_motor()
            # self.read_error()
            # self.clear_error()
            self.read_temperature()
            self.set_speed_and_current()
            # print("err_state: ", self.err_state)
            print(self.color_bool(self.read_success))

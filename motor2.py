# import serial
# import time
# import struct
# import math

# # 串口设置
# SERIAL_PORT = '/dev/ttyCH9344USB3'
# BAUD_RATE = 115200
# PARITY = serial.PARITY_NONE

# TIME_WAIT = 0.06  # 等待时间
# TIME_WAIT_WONLY = 0.06 # 只写不读等待时间

# # 电机ID
# MOTOR_ID = 0x02

# PI = math.pi

# # 最大保护转速（rad/s），转换为°/s
# MAX_SPEED_RPM = 20 * 180 / PI  # 20 rad/s 转换为 °/s

# # 电机1运动状态（全局变量）
# motor_speed = 0.0  # 电机1设置的转速
# motor_status = 0x00  # 电机1运动状态（0x01: 正转, 0x00: 停转, 0xF1: 反转）
# motor_current = 0.00 # 电机1设置的电流
# motor_enable = 0x00 # 电机使能状态
# motor_over_temp = 0x00 # 电机过温保护

# # 电机温度
# T_coil = 0.0  # 电机线圈温度
# T_pcb = 0.0   # 电机驱动板温度

# read_success = False

# # 设置调试开关
# DEBUG = False  # 设置为 True 来启用调试输出

# # 设置串口
# def setup_serial():
#     ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, parity=PARITY, timeout=0)
#     return ser

# # CRC16 校验计算
# def calculate_crc(data):
#     crc = 0xFFFF
#     for byte in data:
#         crc ^= byte
#         for _ in range(8):
#             if crc & 0x0001:
#                 crc = (crc >> 1) ^ 0xA001
#             else:
#                 crc >>= 1
#     return crc & 0xFFFF

# # 发送 Modbus 指令并接收响应
# def send_command(data_to_send, read_bytes=0):
#     ser = setup_serial()
#     crc = calculate_crc(data_to_send)
#     data_to_send.extend(struct.pack('<H', crc))  # 添加 CRC16
#     ser.write(data_to_send)

#     if DEBUG:
#         print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")

#     if read_bytes > 0:
#         delay_precise(TIME_WAIT)
#         response = ser.read(read_bytes)
#         if DEBUG:
#             print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
#         ser.close()
#         return response
#     else:
#         ser.close()
#         delay_precise(TIME_WAIT_WONLY)
#         return None

# # 使能电机
# def enable_motor():
#     global motor_enable
#     data_to_send = bytearray([MOTOR_ID, 0x65, motor_enable])  
#     if DEBUG:
#         print("Enabling motor")
#     send_command(data_to_send)

# # 设置转速和电流
# def set_speed_and_current():
#     global motor_speed, motor_current, motor_status

#     if motor_speed > MAX_SPEED_RPM:
#         motor_speed = MAX_SPEED_RPM
#         print(f"Speed exceeds max limit. Setting to {MAX_SPEED_RPM}°.")
#     elif motor_speed < - MAX_SPEED_RPM:
#         motor_speed =  - MAX_SPEED_RPM
#         print(f"Speed exceeds min limit. Setting to {- MAX_SPEED_RPM}°.")

#     # 转速（°/s）转换为 rad/s
#     speed_rad_s = motor_speed * PI / 180

#     # 转速（rad/s）转为 IEEE 754 浮点数格式
#     speed_bytes = struct.pack('<f', speed_rad_s)

#     # 电流（A）转为 IEEE 754 浮点数格式
#     current_bytes = struct.pack('<f', motor_current)

#     # 更新电机1的转速和状态
#     motor_speed = motor_speed
#     if motor_speed > 0:
#         motor_status = 0x01  # 正转
#     elif motor_speed < 0:
#         motor_status = 0xF1  # 反转
#     else:
#         motor_status = 0x00  # 停转

#     # 构建设置转速和电流指令
#     data_to_send = bytearray([MOTOR_ID, 0x65, 0x06, *speed_bytes, *current_bytes])
#     if DEBUG:
#         print(f"Setting speed: {motor_speed}°/s, current: {motor_current} A")
#         print(f"motor_speed: {motor_speed}°/s")
#         print(f"motor_status: {'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")
    
#     send_command(data_to_send)

# def read_temperature():
#     global T_coil, T_pcb, read_success

#     data_to_send = bytearray([MOTOR_ID, 0x04, 0x00, 0x0D, 0x00, 0x02])
#     response = send_command(data_to_send, 8 + 5)
    
#     if response and len(response) == 13:
#         # CRC 校验
#         crc_received = struct.unpack('<H', response[-2:])[0]
#         crc_calculated = calculate_crc(response[:-2])

#         if crc_received != crc_calculated:
#             if DEBUG:
#                 print("[motor1] CRC 错误！计算值: {:04X}, 接收值: {:04X}".format(crc_calculated, crc_received))
#             read_success = False
#             return None
        
#         if DEBUG:
#             print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
#         T_coil = struct.unpack('<f', response[3:7])[0]
#         T_pcb = struct.unpack('<f', response[7:11])[0]
#         if DEBUG:
#             print("T_coil: ", T_coil)
#             print("T_pcb: ", T_pcb)
#         read_success = True
#         return T_coil, T_pcb
#     else:
#         if DEBUG:
#             print("Error reading temperature.")
#         read_success = False
#         return None

# def delay_precise(duration_s):
#     start = time.perf_counter()
#     end = start + duration_s
#     while True:
#         now = time.perf_counter()
#         if now >= end:
#             break

# # 测试函数
# def test_motor():
#     global motor_speed, motor_current, motor_status, motor_enable

#     print("正转/下降点动")
#     motor_enable = 0x01
#     enable_motor()
#     delay_precise(0.05)  # 等待电机启动
#     # 设置转速和电流
#     motor_speed = 5.0 / PI * 180
#     motor_current = 0.95
#     set_speed_and_current()
#     print(f"电机当前转速：{motor_speed}°/s")
#     print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")
#     delay_precise(0.5)  # 等待设置生效
#     # 设置转速为 0
#     print("设置转速为 0 rad/s, 电流为 0.0 A")
#     motor_speed = 0.0
#     motor_current = 0.0
#     set_speed_and_current()

#     print("反转/上升点动")
#     motor_enable = 0x01
#     enable_motor()
#     delay_precise(0.05)  # 等待电机启动
#     # 设置转速和电流
#     motor_speed = -5.0 / PI * 180
#     motor_current = 0.95
#     set_speed_and_current()
#     print(f"电机当前转速：{motor_speed}°/s")
#     print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")
#     delay_precise(0.5)  # 等待设置生效
#     # 设置转速为 0
#     print("设置转速为 0 rad/s, 电流为 0.0 A")
#     motor_speed = 0.0
#     motor_current = 0.0
#     set_speed_and_current()
    
#     # print("使能电机")
#     motor_enable = 0x01
#     # enable_motor()
    
#     # delay_precise(1)  # 等待电机启动
    
#     # # 设置转速和电流
#     # print("设置转速和电流（15 rad/s, 0.95 A）")
#     # motor_speed = 15.0 / PI * 180
#     # motor_current = 0.95
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")

#     # delay_precise(1)  # 等待设置生效

#     # # 设置负转速和电流
#     # print("设置负转速和电流（-15 rad/s, 0.95 A）")
#     # motor_speed = -15.0 / PI * 180
#     # motor_current = 0.95
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")

#     # delay_precise(1)  # 等待设置生效
    
#     # # 设置转速为 0
#     # print("设置转速为 0 rad/s, 电流为 0.95 A")
#     # motor_speed = 0.0
#     # motor_current = 0.95
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")

#     # delay_precise(1)  # 等待电机启动
    
#     # # 设置转速和电流
#     # print("设置转速和电流（30 rad/s, 0.95 A）")
#     # motor_speed = 30 / PI * 180
#     # motor_current = 0.95
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")

#     # delay_precise(1)  # 等待电机启动
    
#     # # 设置转速和电流
#     # print("设置转速和电流（-30 rad/s, 0.95 A）")
#     # motor_speed = -30 / PI * 180
#     # motor_current = 0.95    
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")

#     # delay_precise(1)  # 等待设置生效

#     # # 设置转速为 0
#     # print("设置转速为 0 rad/s, 电流为 0.95 A")
#     # motor_speed = 0.0
#     # motor_current = 0.0
#     # set_speed_and_current()
#     # print(f"电机当前转速：{motor_speed}°/s")
#     # print(f"电机当前状态：{'正转' if motor_status == 0x01 else '反转' if motor_status == 0xF1 else '停转'}")
    
#     # 读温度 0x0D 0x0E
#     delay_precise(0.05)
#     read_temperature()
#     print("T_coil: ", T_coil)
#     print("T_pcb: ", T_pcb)


# if __name__ == '__main__':
#     test_motor()

# # motor.py
# import serial
# import time
# import struct
# import math

# class MotorController:
#     # 常量值
#     PI = math.pi
#     MAX_SPEED_RPM = 20 * 180 / PI  # 20 rad/s 转换为 °/s
#     TIME_WAIT = 0.06  # 等待时间
#     TIME_WAIT_WONLY = 0.060  # 只写不读等待时间

#     def __init__(self, motor_id, serial_port):
#         # 电机ID和串口
#         self.motor_id = motor_id
#         self.serial_port = serial_port

#         # 电机运动状态
#         self.motor_speed = 0.0  # 电机设置的转速
#         self.motor_status = 0x00  # 电机运动状态（正转/停转/反转）
#         self.motor_current = 0.95  # 电流值
#         self.motor_enable = 0x00  # 电机使能状态

#         # 电机温度
#         self.T_coil = 0.0  # 线圆温度
#         self.T_pcb = 0.0   # 驱动板温度

#         self.read_success = False  # 读取是否成功
#         self.DEBUG = False  # 调试模式

#     # 设置串口
#     def setup_serial(self):
#         return serial.Serial(self.serial_port, baudrate=115200, parity=serial.PARITY_NONE, timeout=0)

#     # CRC16 校验计算
#     def calculate_crc(self, data):
#         crc = 0xFFFF
#         for byte in data:
#             crc ^= byte
#             for _ in range(8):
#                 if crc & 0x0001:
#                     crc = (crc >> 1) ^ 0xA001
#                 else:
#                     crc >>= 1
#         return crc & 0xFFFF

#     # 发送 Modbus 指令并接收响应
#     def send_command(self, data_to_send, read_bytes=0):
#         ser = self.setup_serial()
#         data_to_send.extend(struct.pack('<H', self.calculate_crc(data_to_send)))
#         ser.write(data_to_send)

#         if self.DEBUG:
#             print(f"Sent:    {' '.join(f'{x:02X}' for x in data_to_send)}")

#         if read_bytes > 0:
#             self.delay_precise(self.TIME_WAIT)
#             response = ser.read(read_bytes)
#             if self.DEBUG:
#                 print(f"Rcve:    {' '.join(f'{x:02X}' for x in response)}")
#             ser.close()
#             return response
#         else:
#             ser.close()
#             self.delay_precise(self.TIME_WAIT_WONLY)
#             return None

#     # 使能电机
#     def enable_motor(self):
#         data_to_send = bytearray([self.motor_id, 0x65, self.motor_enable])
#         if self.DEBUG:
#             print("Enabling motor")
#         self.send_command(data_to_send)

#     # 设置转速和电流
#     def set_speed_and_current(self):
#         if self.motor_speed > self.MAX_SPEED_RPM:
#             self.motor_speed = self.MAX_SPEED_RPM
#             print(f"Speed exceeds max limit. Setting to {self.MAX_SPEED_RPM}°.")
#         elif self.motor_speed < -self.MAX_SPEED_RPM:
#             self.motor_speed = -self.MAX_SPEED_RPM
#             print(f"Speed exceeds min limit. Setting to {-self.MAX_SPEED_RPM}°.")

#         # 转速(°/s)转换为 rad/s
#         speed_rad_s = self.motor_speed * self.PI / 180

#         # 转速(弧/s)和电流(A)转换为 IEEE754 浮点数格式
#         speed_bytes = struct.pack('<f', speed_rad_s)
#         current_bytes = struct.pack('<f', self.motor_current)

#         # 更新电机运动状态
#         if self.motor_speed > 0:
#             self.motor_status = 0x01  # 正转
#         elif self.motor_speed < 0:
#             self.motor_status = 0xF1  # 反转
#         else:
#             self.motor_status = 0x00  # 停转

#         data_to_send = bytearray([self.motor_id, 0x65, 0x06, *speed_bytes, *current_bytes])
#         if self.DEBUG:
#             print(f"Setting speed: {self.motor_speed}°/s, current: {self.motor_current} A")
#             print(f"motor_status: {'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")

#         self.send_command(data_to_send)

#     # 读取电机温度
#     def read_temperature(self):
#         data_to_send = bytearray([self.motor_id, 0x04, 0x00, 0x0D, 0x00, 0x02])
#         response = self.send_command(data_to_send, 13)
#         if response and len(response) == 13:
#             crc_received = struct.unpack('<H', response[-2:])[0]
#             crc_calculated = self.calculate_crc(response[:-2])
#             if crc_received == crc_calculated:
#                 self.T_coil = struct.unpack('<f', response[3:7])[0]
#                 self.T_pcb = struct.unpack('<f', response[7:11])[0]
#                 self.read_success = True
#                 return self.T_coil, self.T_pcb
#         self.read_success = False
#         return None

#     # 精确延时
#     def delay_precise(self, duration_s):
#         start = time.perf_counter()
#         while time.perf_counter() < start + duration_s:
#             pass

#     # 测试函数
#     def test_motor(self):
#         print("正转/下降点动")
#         self.motor_enable = 0x01
#         self.enable_motor()
#         self.delay_precise(0.05)  # 等待电机启动
#         self.motor_speed = 5.0 / self.PI * 180
#         self.motor_current = 0.95
#         self.set_speed_and_current()
#         print(f"电机当前转速：{self.motor_speed}°/s")
#         print(f"电机当前状态：{'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")
#         self.delay_precise(0.5)
#         print("设置转速为 0 rad/s, 电流为 0.0 A")
#         self.motor_speed = 0.0
#         self.motor_current = 0.0
#         self.set_speed_and_current()

#         print("反转/上升点动")
#         self.motor_enable = 0x01
#         self.enable_motor()
#         self.delay_precise(0.05)
#         self.motor_speed = -5.0 / self.PI * 180
#         self.motor_current = 0.95
#         self.set_speed_and_current()
#         print(f"电机当前转速：{self.motor_speed}°/s")
#         print(f"电机当前状态：{'正转' if self.motor_status == 0x01 else '反转' if self.motor_status == 0xF1 else '停转'}")
#         self.delay_precise(0.5)
#         print("设置转速为 0 rad/s, 电流为 0.0 A")
#         self.motor_speed = 0.0
#         self.motor_current = 0.0
#         self.set_speed_and_current()

#         # 读取温度
#         self.read_temperature()
#         print("T_coil: ", self.T_coil)
#         print("T_pcb: ", self.T_pcb)



# from motor import MotorController

# # 实例化电机1控制器
# MOTOR_ID = 0x02
# motor = MotorController(MOTOR_ID, '/dev/ttyCH9344USB3')

# # 暴露和以前一致的全局变量
# motor_speed = motor.motor_speed
# motor_status = motor.motor_status
# motor_current = motor.motor_current
# motor_enable = motor.motor_enable
# T_coil = motor.T_coil
# T_pcb = motor.T_pcb
# read_success = motor.read_success

# # 暴露函数
# enable_motor = motor.enable_motor
# set_speed_and_current = motor.set_speed_and_current
# read_temperature = motor.read_temperature

# # 测试函数
# def test_motor():
#     motor.test_motor()

# # 直接运行测试
# if __name__ == '__main__':
#     test_motor()



from motor import MotorController

# 实例化电机2控制器（ID: 0x02，串口: USB3）
motor = MotorController(0x02, '/dev/ttyCH9344USB3')

# 仅导出类实例及必要函数
def test_motor():
    motor.test_motor()

if __name__ == '__main__':
    test_motor()

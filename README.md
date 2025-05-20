# UWC（水下通用控制系统）

## 项目简介

UWC（Universal Water Control）是一套基于串口通信的水下控制系统，支持多种传感器与执行器的并发读取与控制，并以 10Hz 周期上传系统状态数据。适用于水下实验设备、电动抓取机构、智能探测单元等场景。

## 功能模块

项目通过 `uwc.py` 作为主控入口，周期性采集并上传以下设备状态：

- **位移传感器**（displacement）
- **超声波测距仪**（ultrasonic）
- **IMU 姿态传感器**（imu）
- **流速流向仪**（flowmeter）
- **重量传感器**（load）
- **接近开关**（switch）
- **伺服电机**（servo）
- **电机1 - 锁紧电机**（motor1）
- **电机2 - 充电电机**（motor2）

支持指令帧控制伺服角度、电机启停与方向。具备限位保护、温度保护、夹紧保护机制。

## 系统结构

```plaintext
uwc/
├── uwc.py           # 主循环，数据上传与指令处理
├── imu.py           # IMU角度读取
├── displacement.py  # 位移读取
├── ultrasonic.py    # 超声波测距
├── flowmeter.py     # 流速流向读取
├── load.py          # 重量读取与清零
├── switch.py        # 接近开关状态
├── servo.py         # 伺服电机控制
├── motor.py         # 通用电机控制类
├── motor1.py        # 电机1实例封装
├── motor2.py        # 电机2实例封装
```

## 串口映射（建议）

| 设备            | 串口路径               |
| --------------- | ---------------------- |
| 主控上传        | `/dev/ttyCH9344USB0` |
| 伺服电机        | `/dev/ttyCH9344USB1` |
| 电机1（锁紧）   | `/dev/ttyCH9344USB2` |
| 电机2（充电）   | `/dev/ttyCH9344USB3` |
| IMU/负载/开关等 | `/dev/ttyCH9344USB4` |
| 位移传感器      | `/dev/ttyCH9344USB5` |

## 快速开始

1. 连接各类串口设备并确认端口号；
2. 根据需要配置 `DEBUG = True` 开启调试；
3. 启动主控程序：

```bash
python3 uwc.py
```

## 指令协议简述

上传帧格式：

```
[SYN1][SYN2][CMD][LEN][Payload][Checksum][END]
```

- 帧头：`0xAA 0xBB`，帧尾：`0x55`
- Payload 包含 13 个字段（如角度、温度、电机状态等）
- 指令解析详见 `parse_command()` 函数

## 开源许可证

本项目采用 [MIT License](https://opensource.org/licenses/MIT) 开源协议，允许商用、修改、分发和私有使用

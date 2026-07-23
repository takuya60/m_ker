# KER 固件编译、烧录与传输测试手册

本文说明 `firmware` 目录中 M5 主控固件和编码器固件的环境准备、配置、编译、烧录及
USB/WiFi 通信测试方法。

## 1. 目录结构

```text
firmware/
├── M5/                         # KER M5 主控固件
│   ├── include/
│   │   ├── Common.h            # 编码器方向、机械限位和零位配置
│   │   ├── Meta.h              # 固件/硬件版本和 USB VID/PID
│   │   ├── USBStream.h         # USB vendor 数据通路
│   │   ├── SerialStream.h      # 串口数据通路
│   │   ├── WiFiStream.h        # WiFi TCP 数据通路
│   │   ├── WiFiConfig.example.h
│   │   └── WiFiConfig.h        # 本机 WiFi 配置，不提交到 Git
│   ├── src/                    # M5 固件实现
│   └── platformio.ini          # USB/serial/WiFi 构建环境
├── encoder/                    # 单个磁编码器板固件
└── test/
    ├── wifi_stream_test.py     # M5 WiFi schema/数据流测试
    └── 其他 RS-485 测试脚本
```

M5 传输方式在编译时选择。一个固件只使用 USB、serial、WiFi 中的一种，不会在运行时
同时启用或自动切换。

## 2. 安装 PlatformIO

推荐使用 PlatformIO Core：

```bash
python3 -m pip install --user platformio
```

重新打开终端后检查：

```bash
pio --version
```

如果 `pio` 不在 `PATH`：

```bash
python3 -m platformio --version
```

也可以使用 VS Code 的 PlatformIO IDE 插件。

## 3. M5 固件配置

### 3.1 编码器配置

文件：

```text
firmware/M5/include/Common.h
```

`ENCODER_CONFIG` 每个通道包含：

```cpp
{invert, mech_min, mech_max, mech_joint_offset, skip_jump_detect}
```

- `invert`：编码器方向是否反转。
- `mech_min`、`mech_max`：机械角度范围，也用于启动时的正负 360 度修正。
- `mech_joint_offset`：执行 Zero 后该姿态对应的机器人角度。
- `skip_jump_detect`：是否跳过该通道的跳变检测，通常只用于夹爪。

修改这些参数后必须重新编译和烧录 M5 固件。机械 Zero 数据保存在 M5 NVS 中，重新烧录
普通固件通常不会自动清除 NVS。

### 3.2 WiFi 配置

WiFi 环境要求存在：

```text
firmware/M5/include/WiFiConfig.h
```

从模板复制：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker/firmware/M5
cp include/WiFiConfig.example.h include/WiFiConfig.h
```

编辑内容：

```cpp
#pragma once

#define KER_WIFI_SSID     "OpenFlex-Robot"
#define KER_WIFI_PASSWORD "your-password"
#define KER_WIFI_HOSTNAME "openarm-ker"
#define KER_WIFI_PORT     19090
```

- M5/ESP32 通常使用 2.4 GHz WiFi。
- `KER_WIFI_HOSTNAME=openarm-ker` 对应 `openarm-ker.local`。
- `KER_WIFI_PORT` 必须和 ROS 或测试脚本一致。
- `WiFiConfig.h` 已被 `.gitignore` 排除，不会提交密码。
- 缺少 `WiFiConfig.h` 时，WiFi 固件会明确编译失败。

## 4. 编译命令

进入 M5 工程：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker/firmware/M5
```

### 4.1 USB 固件

```bash
pio run -e usb
```

生成文件：

```text
.pio/build/usb/firmware.bin
```

### 4.2 串口固件

```bash
pio run -e serial
```

### 4.3 WiFi 固件

确认 `include/WiFiConfig.h` 已配置：

```bash
pio run -e wifi
```

生成文件：

```text
.pio/build/wifi/firmware.bin
```

### 4.4 同时检查 USB 和 WiFi 环境

```bash
pio run -e usb -e wifi
```

该命令只是连续编译两个独立固件，不代表生成 USB/WiFi 同时运行的固件。

### 4.5 清理后重新编译

```bash
pio run --target clean
pio run -e usb
```

只清理 WiFi 环境：

```bash
pio run -e wifi --target clean
```

## 5. 烧录命令

使用支持数据传输的 USB 线连接 M5。必要时按住复位键约 3 秒进入下载模式。

### 5.1 USB 固件

```bash
pio run -e usb --target upload
```

### 5.2 串口固件

```bash
pio run -e serial --target upload
```

### 5.3 WiFi 固件

```bash
pio run -e wifi --target upload
```

指定端口：

```bash
pio run -e wifi --target upload --upload-port /dev/ttyACM0
```

查看可用端口：

```bash
pio device list
```

## 6. 串口日志和屏幕状态

WiFi 和 serial 环境启用了 USB CDC，可尝试：

```bash
pio device monitor -b 115200
```

如果程序没有串口日志，以 M5 屏幕状态为准：

- `WIFI --`：尚未获得 IP。
- `WIFI <IP> --`：已联网，尚无 TCP 客户端。
- `WIFI <IP> C`：已有 TCP 客户端。
- `USB CONNECTED`：USB vendor 客户端已连接。

## 7. WiFi 数据测试

测试脚本不依赖 ROS，也不需要第三方 Python 包：

```text
firmware/test/wifi_stream_test.py
```

脚本顶部集中定义了默认测试配置：

```python
DEFAULT_HOST = '192.168.3.114'
DEFAULT_PORT = 19090
DEFAULT_DURATION_S = 10.0
DEFAULT_PRINT_RATE_HZ = 2.0
DEFAULT_CONNECT_TIMEOUT_S = 3.0
DEFAULT_SOCKET_TIMEOUT_S = 0.2
DEFAULT_SCHEMA_TIMEOUT_S = 5.0
DEFAULT_DATA_TIMEOUT_S = 3.0
DEFAULT_EXPECT_FW_PREFIX = '3.1.0-wifi'
```

修改这些常量后，可以不带参数直接运行：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker
python3 firmware/test/wifi_stream_test.py
```

命令行参数只是临时覆盖默认值，不是必须填写。

测试前确认：

1. M5 已烧录 `wifi` 环境。
2. M5 与测试主机处于同一局域网。
3. M5 屏幕已显示有效 IPv4 地址。
4. 没有 ROS driver 或其他程序占用唯一 TCP 客户端连接。

使用 mDNS：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker
python3 firmware/test/wifi_stream_test.py openarm-ker.local
```

使用屏幕显示的 IPv4：

```bash
python3 firmware/test/wifi_stream_test.py 192.168.1.105
```

运行 10 秒，每秒最多打印 2 次：

```bash
python3 firmware/test/wifi_stream_test.py 192.168.1.105 \
  --duration 10 \
  --print-rate 2
```

检查固件版本：

```bash
python3 firmware/test/wifi_stream_test.py openarm-ker.local \
  --expect-fw-prefix 3.1.0-wifi
```

常用参数：

```text
host                     M5 主机名或 IPv4，默认 openarm-ker.local
--port                    TCP 端口，默认 19090
--duration                测试秒数，0 表示持续运行
--print-rate              每秒最多打印次数
--connect-timeout         TCP 建连超时
--schema-timeout          PING/schema 超时
--data-timeout            有效数据超时
--expect-fw-prefix        固件版本前缀检查
```

脚本流程：

```text
TCP connect
  -> STANDBY
  -> PING
  -> 解析 FW/HW/schema
  -> STREAM
  -> 校验 XOR checksum
  -> 打印 angles、error_mask 和接收频率
  -> 退出时发送 STANDBY
```

正常输出示例：

```text
[OK] Connected to 192.168.1.105:19090
[OK] FW=3.1.0-wifi HW=3.0.0 updated=2026-07-23
[OK] Fields: timestamp[1], angles[16], errors[16]
[DATA] hz=98.7 error_mask=0xFF80 angles_deg=[+0.10, ...]
```

## 8. ROS WiFi 测试

完成独立脚本测试后，再启动 ROS driver：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash

ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/openflex/openflex_all/openflex_ws/src/m_ker/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=wifi \
  -p wifi_host:=openarm-ker.local \
  -p wifi_port:=19090
```

检查：

```bash
ros2 service call /ker/ping_wifi std_srvs/srv/Trigger
ros2 topic echo /ker/metadata --once
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/error_mask --once
```

直接通过 WiFi 启动 OpenArmX fake hardware 和 RViz 遥操：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py \
  transport:=wifi \
  wifi_host:=192.168.3.114 \
  wifi_port:=19090
```

当前 `openflex_ker_rviz_sim.launch.py` 已将 `192.168.3.114:19090` 定义为默认 WiFi
端点，因此同一设备可以简化为：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

仍可使用命令行参数覆盖文件顶部默认值。

测试脚本和 ROS driver 不能同时连接，因为当前固件只接受一个 TCP 客户端。

## 9. 编码器板固件

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker/firmware/encoder
pio project config
pio run
```

烧录器连接和 ID 设置以 `firmware/encoder/README.md` 为准。不要把 M5 主控固件误烧到
编码器板，也不要把编码器固件烧到 M5。

## 10. 常见问题

### `pio: command not found`

```bash
python3 -m pip install --user platformio
python3 -m platformio run -e wifi
```

### WiFi 编译提示缺少 `WiFiConfig.h`

```bash
cp include/WiFiConfig.example.h include/WiFiConfig.h
```

填写真实 SSID 和密码后重新编译。

### M5 一直显示 `WIFI --`

- 确认使用 2.4 GHz WiFi。
- 检查 SSID、密码和信号强度。
- 确认路由器没有使用企业认证或客户端隔离。

### `openarm-ker.local` 无法解析

直接使用 M5 屏幕显示的 IPv4 地址。

### TCP 连接成功但没有数据

- 确认没有第二个客户端。
- 确认客户端先 PING/schema，再发送 STREAM。
- 检查 M5 是否因 Jump Detection 回到 STANDBY。

### checksum 错误

- 检查 WiFi 是否频繁断开。
- 确认连接的是 KER 的 `19090` 端口。
- 重启 M5 后重新测试。

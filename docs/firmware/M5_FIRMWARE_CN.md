# KER M5 固件使用手册

本文说明 KER M5 主控固件的配置、编译、烧录、标定、屏幕操作和通信测试。

## 1. 固件简介

M5 固件从 RS-485 总线读取 16 路磁编码器，将原始角度转换为 KER 关节角度，再通过
USB、串口或 WiFi 发送给主机。

```text
16 路编码器
  -> RS-485
  -> M5 角度处理、Zero 和机械偏移
  -> USB / serial / WiFi
  -> 测试脚本或 ROS 2 driver
```

传输方式在编译时确定。一个固件只启用一种传输方式，不能在运行时同时使用 USB 和
WiFi。三种固件的数据字段一致：

| 字段 | 类型 | 数量 | 说明 |
|---|---|---:|---|
| `timestamp` | `uint32` | 1 | M5 时间戳，单位为微秒。 |
| `angles` | `float` | 16 | 16 路关节角度，单位为度。 |
| `errors` | `bool` | 16 | 各通道读取错误状态。 |

固件采集任务以约 1 ms 周期运行。实际传输频率还会受到 RS-485、USB/WiFi 和主机处理
频率影响。

## 2. 配置说明

M5 工程位于：

```text
firmware/M5/
```

主要文件：

| 文件 | 作用 |
|---|---|
| `platformio.ini` | USB、serial、WiFi 三种编译环境。 |
| `include/Common.h` | 引脚、16 路编码器方向、机械范围和偏移。 |
| `include/WiFiConfig.h` | WiFi 配置。 |
| `include/Meta.h` | 固件版本、硬件版本、USB VID/PID。 |
| `src/main.cpp` | 采集任务、GUI 任务、数据发送和 Zero 保存。 |
| `src/GUIHandler.cpp` | 屏幕柱状图和触摸按钮。 |
| `src/USBStream.cpp` | USB vendor 通信。 |
| `src/WiFiStream.cpp` | WiFi TCP 服务。 |

### 2.1 WiFi 配置

首次编译 WiFi 固件前执行：

```bash
cd firmware/M5
cp include/WiFiConfig.example.h include/WiFiConfig.h
```

编辑 `include/WiFiConfig.h`：

```cpp
#pragma once

#define KER_WIFI_SSID     "你的WiFi名称"
#define KER_WIFI_PASSWORD "你的WiFi密码"
#define KER_WIFI_HOSTNAME "openarm-ker"
#define KER_WIFI_PORT     19090
```

- M5 使用 2.4 GHz WiFi。
- 主机名 `openarm-ker` 通常可通过 `openarm-ker.local` 访问。
- TCP 端口必须与测试脚本和 ROS driver 一致。
- 当前协议没有加密和身份认证，应在可信局域网中使用。
- WiFi 固件同时只接受一个 TCP 客户端。

### 2.2 编码器配置

`include/Common.h` 中每个通道的格式为：

```cpp
{invert, mech_min, mech_max, mech_joint_offset}
```

| 参数 | 说明 |
|---|---|
| `invert` | 是否反转编码器方向。 |
| `mech_min` | 关节机械最小角度，单位为度。 |
| `mech_max` | 关节机械最大角度，单位为度。 |
| `mech_joint_offset` | 执行 Zero 后，该姿态对应的机器人关节角度。 |


## 3. 编译与烧录

### 3.1 安装 PlatformIO

```bash
python3 -m pip install --user platformio
pio --version
```

如果终端找不到 `pio`，可将下面命令中的 `pio` 替换为 `python3 -m platformio`。

进入工程：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker/firmware/M5
```

### 3.2 编译

```bash
# USB 固件
pio run -e usb

# WiFi 固件
pio run -e wifi

# 同时检查两种独立固件
pio run -e usb -e wifi
```

输出文件分别位于：

```text
.pio/build/usb/firmware.bin
.pio/build/wifi/firmware.bin
```

### 3.3 烧录（只需烧录一种）

用支持数据传输的 USB 线连接 M5。按住复位键 3 秒进入烧录模式。进入M5文件夹下

```bash
# 烧录 USB 固件
pio run -e usb --target upload

# 烧录 WiFi 固件
pio run -e wifi --target upload
```

指定烧录端口：

```bash
pio device list
pio run -e wifi --target upload --upload-port /dev/ttyACM0
```

烧录完成后按一次复位键启动固件。

## 4. 标定

### 4.1 标定前准备

- 确认 16 个编码器 ID 与关节顺序正确。
- 确认各关节在机械范围内，并且编码器安装牢固。
- 标定时不要运行遥操 bridge，不要向真实机械臂发送控制命令。
- 将 KER 放到预定的标准标定姿态。

通道顺序：

| 通道 | 对应关节 |
|---:|---|
| 1-7 | 右臂 J1-J7 |
| 8 | 右夹爪 |
| 9-15 | 左臂 J1-J7 |
| 16 | 左夹爪 |

### 4.2 Zero All

1. 将 KER 调整到标准标定姿态。
2. 确认屏幕 16 路柱状图没有显示 `ERR`。
3. 点击左下角 `Zero Reset (All)`。
4. 在确认框中点击 `YES`。
5. 等待角度刷新，然后缓慢移动各关节检查方向。

Zero 数据保存在 M5 的 NVS 中，正常断电重启后不会丢失。

### 4.3 单通道 Zero

如果只重新安装了一个编码器，无需重新标定全部通道：

1. 点击对应柱状图，使其变为亮蓝色。
2. 点击左下角 `Confirm Zero`。
3. 在确认框中点击 `YES`。

### 4.4 标定后检查

- 重启 M5，确认 Zero 仍然有效。
- 缓慢移动每个关节，确认角度方向正确且连续。
- 确认角度不会无故变化约 360 度。
- 确认机械极限附近的数值符合 `mech_min` 和 `mech_max`。
- 连接测试脚本或 ROS 后检查 `error_mask`；完整 16 路设备应为 `0`。

如果方向相反，修改对应通道的 `invert` 后重新编译烧录。

## 5. 屏幕操作

| 显示或按钮 | 作用 |
|---|---|
| 1-16 柱状图 | 显示各通道角度；`ERR` 表示读取异常。 |
| `Zero Reset (All)` | 标定全部通道。 |
| `Confirm Zero` | 标定已选中的橙色通道。 |
| `START` | 进入 STREAM，开始发送数据。 |
| `STOP` | 返回 STANDBY，停止发送数据。 |
| `WIFI <IP> C` | WiFi 已连接，并有 TCP 客户端。 |
| `USB CONNECTED` | USB 客户端已连接。 |

进入 STREAM 后柱状图继续实时显示，只有右下角按钮由 `START` 变为 `STOP`。

## 6. 数据测试

### 6.1 WiFi 独立测试

WiFi 测试脚本不依赖 ROS：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker
python3 firmware/test/wifi_stream_test.py 192.168.3.114
```

也可以使用 mDNS：

```bash
python3 firmware/test/wifi_stream_test.py openarm-ker.local
```

脚本会执行 TCP 连接、PING/schema、STREAM、checksum 校验，并打印角度、错误位和接收
频率。测试时不要同时启动 ROS driver，因为 WiFi 固件只允许一个客户端。

### 6.2 ROS 测试

启动 KER driver 后检查：

```bash
ros2 service call /ker/ping_wifi std_srvs/srv/Trigger
ros2 topic echo /ker/metadata --once
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/error_mask --once
```

默认 ROS driver 以最高约 100 Hz 发布 `/ker/joint_states`，bridge 默认以 50 Hz 输出控制
命令。

## 7. 常见问题

### WiFi 编译提示缺少 `WiFiConfig.h`

```bash
cd firmware/M5
cp include/WiFiConfig.example.h include/WiFiConfig.h
```

填写真实 WiFi 信息后重新编译。

### M5 一直显示 `WIFI --`

- 确认使用 2.4 GHz WiFi。
- 检查 SSID、密码和信号强度。
- 确认路由器没有启用客户端隔离。

### `openarm-ker.local` 无法访问

直接使用 M5 屏幕显示的 IPv4 地址。

### WiFi 已连接但没有数据

- 确认只运行了一个客户端。
- 确认 M5 已进入 STREAM。
- 确认客户端完成 PING/schema 后发送了 STREAM 命令。
- 检查端口是否为 `19090`。

### USB 提示权限不足

临时测试可以使用管理员权限确认问题，但正常使用应配置 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 M5 后再启动 driver。

### 角度方向相反

检查对应通道的 `invert`。如果只是 KER 与 OpenArmX 坐标系定义不同，则应在 ROS
`joint_scales` 中修正，不要修改两次。

### 角度跳变约 360 度

检查 Zero 姿态、`mech_min`、`mech_max` 和编码器安装方向。滤波只能减小突变影响，不能
代替正确标定。

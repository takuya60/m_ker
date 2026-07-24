# KER 编码器固件使用手册

本文说明 KER 磁编码器板的 ID、连接、固件烧录和 RS-485 通信测试。

## 1. 编码器简介

每块编码器板由 ATtiny1616 读取 TLE5012B 磁编码器，并通过 RS-485 向 M5 返回角度。

```text
磁体转动
  -> TLE5012B
  -> ATtiny1616
  -> RS-485 2 Mbps
  -> M5 主控
```

主要参数：

| 项目 | 当前配置 |
|---|---|
| MCU | ATtiny1616，20 MHz |
| 角度传感器 | TLE5012B |
| 传感器接口 | SSC |
| 总线 | RS-485，2 Mbps |
| 数据包 | 4 字节 |
| 设备 ID | 固件编译时指定 |

编码器固件只负责读取并返回原始角度。关节方向、机械范围、机械偏移和 Zero 标定都由
M5 固件处理。

## 2. ID 与关节对应

协议支持 ID 0-31，当前 KER 使用 ID 1-16：

| 编码器 ID | 对应关节 |
|---:|---|
| 1-7 | 右臂 J1-J7 |
| 8 | 右夹爪 |
| 9-15 | 左臂 J1-J7 |
| 16 | 左夹爪 |

同一条 RS-485 总线上不能存在重复 ID。重复 ID 会导致多个编码器同时回复，产生数据冲突
或角度异常。

更换编码器板时，应给新板烧录与原位置相同的 ID。例如更换右臂 J5，应烧录 ID 5。

## 3. 连接与烧录器

编码器使用 USB 转 UPDI 烧录器，例如 Adafruit UPDI Friend 或兼容的 serialUPDI 工具。

烧录时连接：

- UPDI 烧录信号。
- 编码器板电源和 GND。
- 烧录器与编码器板必须共地。

接入 KER 时连接 RS-485 的 A、B 和 GND。A/B 接反通常会导致扫描不到 ID 或完全没有回复。

## 4. 编译与烧录

编码器工程位于：

```text
firmware/encoder/
```

### 4.1 安装 PlatformIO

```bash
python3 -m pip install --user platformio
pio --version
```

### 4.2 指定 ID 并烧录

进入工程：

```bash
cd ../firmware/encoder
```
烧录时可以使用提供的烧录工具，也可以自己通过命令行烧录

使用 `DEVICE_ID` 参数指定本次烧录的编码器 ID：

```bash
PLATFORMIO_BUILD_FLAGS="-DDEVICE_ID=5" \
  pio run --target upload --upload-port /dev/ttyUSB0
```

上面的命令会烧录 ID 5。烧录其他位置时只修改 `DEVICE_ID`：

```bash
# 右臂 J1
PLATFORMIO_BUILD_FLAGS="-DDEVICE_ID=1" \
  pio run --target upload --upload-port /dev/ttyUSB0

# 左夹爪
PLATFORMIO_BUILD_FLAGS="-DDEVICE_ID=16" \
  pio run --target upload --upload-port /dev/ttyUSB0
```

查看烧录器端口：

```bash
pio device list
```

`firmware/encoder/firmware/` 中保存了按 ID 生成的 `firmware_IDxx.hex`，用于批量生产、备份
或使用其他烧录工具写入。日常开发建议使用上述 PlatformIO 命令从源码编译，避免烧错 ID。

每烧录一块板都必须显式指定 `DEVICE_ID`，不要依赖 `platformio.ini` 中可能残留的默认值。

## 5. 通信测试

测试需要 USB-RS485 模块，并安装 PySerial：

```bash
python3 -m pip install pyserial
```

测试脚本位于：

```text
firmware/test/
```

### 5.1 扫描 ID

编辑 `firmware/test/scan_id.py`：

```python
PORT = '/dev/ttyUSB0'
BAUD = 2000000
```

运行：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker
python3 firmware/test/scan_id.py
```

脚本扫描 ID 1-16。每块编码器应只出现一个预期 ID；如果没有响应，检查供电、A/B、端口和
烧录 ID。如果出现重复或不稳定响应，应断开其他编码器后逐块检查。

### 5.2 指定 ID 读取角度

编辑 `firmware/test/test.py`：

```python
PORT = '/dev/ttyUSB0'
BAUD = 2000000
DEVICE_ID = 5
```

运行：

```bash
python3 firmware/test/test.py
```

脚本会持续请求指定 ID，并打印：

```text
Node ID | Raw | Angle
```

缓慢转动磁体或关节，确认角度连续变化且没有长时间超时。完成测试后按 `Ctrl+C` 退出。

测试单块编码器时，建议先断开其他编码器，确认单板正常后再接回完整 RS-485 总线。

## 6. 常见问题

### 找不到 UPDI 烧录器

- 检查 USB 线是否支持数据传输。
- 使用 `pio device list` 确认端口。
- 检查当前用户是否有串口访问权限。
- Linux 常见端口为 `/dev/ttyUSB0`，Windows 常见端口为 `COMx`。

### 烧录失败

- 检查 UPDI、供电和 GND。
- 确认没有同时使用不兼容的外部供电。
- 确认命令中的烧录器端口正确。

### 扫描不到 ID

- 检查 RS-485 A/B 是否接反。
- 确认 USB-RS485 模块支持 2 Mbps。
- 确认脚本端口和波特率正确。
- 断开其他编码器，只测试当前板。
- 重新确认烧录时使用的 `DEVICE_ID`。

### 指定 ID 没有回复

确认 `test.py` 中的 `DEVICE_ID` 与实际烧录 ID 一致。如果扫描脚本找到的是其他 ID，应重新
烧录正确 ID，而不是修改关节映射迁就错误 ID。

### 角度不变化或跳变

- 检查磁体与 TLE5012B 的距离、同轴度和固定状态。
- 检查传感器板供电是否稳定。
- 检查 RS-485 接线和总线干扰。
- 先使用单板测试排除 ID 冲突。

### 接入 M5 后显示 `ERR`

先用 `scan_id.py` 和 `test.py` 确认编码器板正常，再检查 M5 到编码器链路的 A/B、GND、
供电和对应 ID。修复通信后再执行 M5 Zero 标定。

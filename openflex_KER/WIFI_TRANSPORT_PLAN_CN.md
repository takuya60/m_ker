# OpenFlex KER WiFi 传输改造计划

## 1. 项目目标

在保留现有 USB 传输方式的基础上，为 OpenFlex KER 增加 WiFi 局域网传输，并同步更新 ROS 2 驱动和 RViz2 面板。

目标能力：

1. M5 和机器人主机连接到同一个局域网后，机器人主机可以通过 WiFi 接收 KER 角度数据。
2. WiFi 模式支持与 USB 模式相同的 PING、STANDBY、STREAM、ZERO ALL 和 ZERO MASK 命令。
3. RViz2 面板可以分别执行 USB Ping 和 WiFi Ping，并显示固件版本、硬件版本、更新时间、网络端点和延迟。
4. 固件在编译和烧录时选择 USB 或 WiFi，运行时不自动切换传输方式。
5. USB 与 WiFi 使用相同的数据 schema 和 ROS 2 话题，上层控制桥不感知传输方式。

## 2. 本阶段不包含

- USB 与 WiFi 运行时自动切换。
- M5 同时向多个机器人主机推流。
- 跨公网通信。
- Web 配网或手机配网界面。
- TLS 加密和用户认证。
- 修改 KER 角度、标定和 OpenFlex 控制映射。

本阶段按可信局域网设计。部署时应使用机器人专用或受控网络，不能将 M5 TCP 端口直接暴露到公网。

## 3. 总体架构

### 3.1 USB 模式

```text
M5 USBVendor
  -> openflex_teleop_ker/KERStream(usb)
  -> /ker/joint_states
  -> /ker/{left,right}_arm/joint_command
  -> openflex_ker_arm_bridge
  -> OpenFlex controllers
```

### 3.2 WiFi 模式

```text
M5 WiFi TCP Server :19090
  -> 局域网
  -> openflex_teleop_ker/KERStream(wifi)
  -> /ker/joint_states
  -> /ker/{left,right}_arm/joint_command
  -> openflex_ker_arm_bridge
  -> OpenFlex controllers
```

USB 和 WiFi 只替换最底层传输，上层角度处理、故障位图、安全接管和 controller 话题保持不变。

## 4. 网络协议选择

### 4.1 使用 TCP

WiFi 数据传输使用 TCP，不使用 UDP。

原因：

- KER 协议是双向的，主机需要发送 Ping、Stream 和 Zero 命令。
- Ping schema 不能丢失或乱序。
- Zero 命令必须可靠送达，不能重复或部分接收。
- 当前主机解析器已经使用字节缓冲区处理数据帧，容易扩展到 TCP。
- 当前数据量较小，TCP 的额外开销可以接受。

默认端口：

```text
19090/TCP
```

EXO 已使用 `19091`，KER WiFi 使用 `19090` 避免冲突。

### 4.2 数据帧保持兼容

M5 发往主机的数据帧保持官方格式：

```text
Stream: 0xA5 0x5A + schema payload + XOR checksum
Ping:   0xA5 0x50 + metadata + fields + snapshot
```

USB 和 WiFi 共用：

- 字段定义。
- 数据类型。
- 字节序。
- Stream 帧头。
- XOR 校验。
- Ping metadata 格式。

这样主机端只增加 socket 读写，不复制角度协议解析逻辑。

### 4.3 WiFi 命令增加帧封装

TCP 没有消息边界，一次 `recv()` 可能收到半条命令或多条命令。不能直接把 TCP
`recv()` 的缓冲区交给当前 `onCommand()`。

WiFi 命令使用固定帧：

```text
byte 0: 0xA5
byte 1: 0x43                 # Command frame
byte 2: command
byte 3: argument high byte
byte 4: argument low byte
byte 5: XOR(byte 2..4)
```

命令定义：

| 命令 | 值 | 参数 |
|---|---:|---|
| PING | `0x00` | `0x0000` |
| STANDBY | `0x01` | `0x0000` |
| STREAM | `0x02` | `0x0000` |
| ZERO ALL | `0x03` | `0x0000` |
| ZERO MASK | `0x04` | 16 位通道 mask |

`WiFiStream` 负责缓存、寻找帧头、校验和拆包，再将兼容的 3 字节 payload
`[command, arg_hi, arg_lo]` 交给现有固件命令回调。

## 5. M5 网络行为

### 5.1 WiFi 连接

WiFi 固件启动后：

1. 从编译配置读取 SSID 和密码。
2. 以 Station 模式连接局域网。
3. 默认通过 DHCP 获取 IP。
4. 注册 mDNS 名称 `openarm-ker.local`。
5. 启动 TCP Server，监听 `19090`。
6. 只允许一个活动客户端。
7. 客户端断开后回到等待连接状态。
8. WiFi 断开后采用非阻塞方式重新连接。

主机优先使用：

```text
openarm-ker.local:19090
```

mDNS 不可用时可以在 ROS YAML 中填写 M5 的实际 IP。

### 5.2 M5 屏幕状态

WiFi 固件在 STANDBY 界面显示：

- WiFi SSID。
- 当前 IP。
- mDNS 名称。
- TCP 端口。
- 主机是否已连接。
- 当前模式：STANDBY 或 STREAM。

连接失败时显示明确状态，不阻塞编码器采集和 M5 触摸界面。

### 5.3 连接与推流规则

- 没有 TCP 客户端时不发送数据。
- 收到 PING 后进入 STANDBY 并返回 metadata/schema。
- 收到 STREAM 后开始发送数据。
- 客户端断开后立即停止网络发送并回到 STANDBY。
- 只保留最新 `SensorSnapshot`，不积压历史角度帧。
- TCP socket 开启 `TCP_NODELAY`，减少小包延迟。

## 6. 烧录时选择 USB 或 WiFi

修改：

```text
ker/firmware/M5/platformio.ini
```

保留 USB 环境：

```ini
[env:usb]
build_flags =
    -D USE_USB
    -D CONFIG_TINYUSB_VENDOR_ENABLED=1
```

新增 WiFi 环境：

```ini
[env:wifi]
build_flags =
    -D USE_WIFI
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
```

现有串口环境补充 `-D USE_SERIAL`。编译时保证 `USE_USB`、`USE_WIFI`、`USE_SERIAL`
三者只能定义一个；同时定义多个或全部未定义时，编译阶段直接报错。串口方式虽然不在
本次新增功能范围内，但不能因 WiFi 改造而失效。

烧录 USB 数据传输固件：

```bash
cd ker/firmware/M5
pio run -e usb --target upload
```

烧录 WiFi 数据传输固件：

```bash
cd ker/firmware/M5
pio run -e wifi --target upload
```

WiFi 固件仍可通过 USB CDC 烧录和查看日志，但 KER 数据只通过 WiFi TCP 发送。

## 7. WiFi 配置文件

新增模板：

```text
ker/firmware/M5/include/WiFiConfig.example.h
```

开发者创建本机配置：

```text
ker/firmware/M5/include/WiFiConfig.h
```

配置内容：

```cpp
#pragma once

#define KER_WIFI_SSID     "robot-network"
#define KER_WIFI_PASSWORD "replace-me"
#define KER_WIFI_HOSTNAME "openarm-ker"
#define KER_WIFI_PORT     19090
```

`WiFiConfig.h` 必须加入 `.gitignore`，避免将现场 WiFi 密码提交到仓库。仓库只保存
`WiFiConfig.example.h`。

如果现场网络不支持 mDNS，可在 ROS 配置中填写 DHCP 地址；第一阶段不实现静态 IP
写入固件。

## 8. 固件代码改造

### 8.1 新增公共协议类型

新增：

```text
ker/firmware/M5/include/StreamTypes.h
```

将目前重复定义在 `USBStream.h` 和 `SerialStream.h` 中的内容移动到该文件：

- `Type`
- `FieldDef`
- `CommandCallback`

USB、Serial、WiFi 三种 Stream 共用同一套类型，避免依赖头文件包含顺序。

### 8.2 新增 WiFiStream

新增：

```text
ker/firmware/M5/include/WiFiStream.h
ker/firmware/M5/src/WiFiStream.cpp
```

接口与 `USBStream` 保持一致：

```cpp
add()
begin()
mounted()
hasError()
set()
send()
recv()
onCommand()
sendPingResponse()
```

WiFi 特有职责：

- 管理 `WiFiClient` 和 `WiFiServer`。
- 非阻塞重连。
- TCP 命令帧缓存与解析。
- 处理 TCP 拆包和合包。
- 客户端断开检测。
- 暴露 IP、RSSI、hostname 和 client 状态给 GUI。

### 8.3 修改主程序传输选择

修改：

```text
ker/firmware/M5/src/main.cpp
```

传输实例选择改为：

```cpp
#if defined(USE_USB)
  #include "USBStream.h"
  USBStream stream;
#elif defined(USE_WIFI)
  #include "WiFiStream.h"
  WiFiStream stream;
#elif defined(USE_SERIAL)
  #include "SerialStream.h"
  SerialStream stream(Serial);
#else
  #error "Select exactly one KER transport"
#endif
```

保持现有 schema 注册、命令回调、Zero、NVS 和数据填充逻辑不变。

### 8.4 修改 GUI

修改：

```text
ker/firmware/M5/include/GUIHandler.h
ker/firmware/M5/src/GUIHandler.cpp
```

增加 WiFi 状态显示，不改变 Zero、START、STOP 和 Jump Detection 的触摸区域。

### 8.5 修改版本信息

修改：

```text
ker/firmware/M5/include/Meta.h
```

USB 和 WiFi 固件使用可区分版本，例如：

```text
2.1.0-of-usb
2.1.0-of-wifi
```

版本字符串必须控制在 Ping 协议的 16 字节字段内。

## 9. ROS 2 主机驱动改造

### 9.1 KERStream 增加 WiFi transport

修改：

```text
openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_stream.py
```

新增参数：

```text
transport: usb | serial | wifi
wifi_host: openarm-ker.local
wifi_port: 19090
connect_timeout_s: 3.0
socket_timeout_s: 0.02
```

WiFi 模式：

- 使用 `socket.create_connection()` 建立 TCP。
- 接收数据继续进入现有 `_buffer` 和 `_read_packets()`。
- `send_command()` 在 WiFi 模式封装命令帧。
- `close()` 关闭 socket。
- 断线后设置 `is_connected=false`，由现有驱动重连机制恢复。

### 9.2 Driver 参数

修改：

```text
openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_driver_node.py
openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml
openflex_KER/openflex_teleop_ker/launch/openflex_ker_teleop.launch.py
```

新增配置：

```yaml
transport: usb
wifi_host: openarm-ker.local
wifi_port: 19090
wifi_connect_timeout_s: 3.0
```

启动 WiFi 模式：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py transport:=wifi
```

ROS 2 输出话题保持不变。

## 10. USB Ping 与 WiFi Ping

### 10.1 Ping 的含义

这里的 WiFi Ping 不是 ICMP `ping`。它执行完整的 KER 协议 PING：

1. 建立 USB 或 TCP 连接。
2. 发送 KER PING 命令。
3. 读取 firmware、hardware、updated 和 schema。
4. 计算往返时间。
5. 恢复 STREAM。

### 10.2 ROS 服务设计

保留当前兼容服务：

```text
/ker/ping                 std_srvs/srv/Trigger
```

它 Ping 当前配置的活动 transport。

新增：

```text
/ker/ping_usb             std_srvs/srv/Trigger
/ker/ping_wifi            std_srvs/srv/Trigger
```

WiFi Ping 使用 YAML 中的 `wifi_host` 和 `wifi_port`。服务响应包含：

```text
transport
endpoint
hardware version
firmware version
updated
round-trip latency
error reason
```

同一时间只允许一个 Ping 或活动重连操作，使用锁避免 USB/socket 被多个回调同时操作。

## 11. RViz2 Panel 改造

修改：

```text
openflex_KER/openflex_tool/openflex_ker_joint_panel/
  include/openflex_ker_joint_panel/joint_angle_panel.hpp
  src/joint_angle_panel.cpp
```

面板连接区域增加：

- USB/WiFi 分段选择。
- WiFi 主机显示，默认 `openarm-ker.local`。
- WiFi 端口显示，默认 `19090`。
- USB Ping 按钮。
- WiFi Ping 按钮。
- 独立的 Ping 结果状态栏。

显示内容：

```text
USB Ping 成功 | HW ... | FW ... | Updated ...
WiFi Ping 成功 | openarm-ker.local:19090 | 12 ms | HW ... | FW ...
```

失败状态区分：

- ROS 服务未启动。
- hostname 无法解析。
- TCP 连接超时。
- WiFi 已连接但 KER PING 无响应。
- schema 格式错误。
- USB 设备不存在。
- 固件传输模式不匹配。

面板保存 RViz 配置时保留最后选择的传输模式和显示端点。

## 12. 故障安全

WiFi 断开时：

1. `KERStream` 立即标记断线。
2. Driver 停止发布新的 KER 目标。
3. `openflex_ker_arm_bridge` 在 `target_timeout_s` 后停止发送 controller 命令。
4. Driver 按 `reconnect_interval_s` 尝试重连。
5. 重连成功后重新执行 Ping/schema 和 STREAM。
6. 控制桥不重新执行突然归零，继续使用安全接管状态和最新机器人反馈。

需要重点验证当前 `0.25 s` 目标超时在 WiFi 抖动下不会误触发，同时也不能为了减少
误触发而放宽到不安全的时间。

## 13. 实施阶段

### 阶段 1：协议公共化

- 提取 `StreamTypes.h`。
- 保证 USB 和 Serial 固件行为不变。
- 为主机数据解析增加 TCP 拆包测试。

完成标准：USB 和 Serial 编译、Ping、Stream、Zero 全部回归通过。

### 阶段 2：M5 WiFiStream

- 增加 WiFi 编译环境和私有配置模板。
- 实现 TCP Server、命令解析和数据发送。
- 增加 M5 网络状态显示。

完成标准：主机可通过 TCP 获取 Ping schema 和连续角度数据。

### 阶段 3：ROS 2 WiFi 驱动

- `KERStream` 增加 socket transport。
- Driver、YAML、launch 增加 WiFi 参数。
- 实现断线和自动重连。

完成标准：WiFi 与 USB 发布完全相同的 ROS 2 消息。

### 阶段 4：双模式 Ping 和 RViz Panel

- 增加 USB/WiFi Ping 服务。
- 更新面板控件和状态显示。
- 保存 RViz 连接配置。

完成标准：面板能够明确识别成功、超时、固件模式不匹配和设备不存在。

### 阶段 5：系统验证

- USB 回归。
- WiFi 长时间推流。
- 网络断开恢复。
- 控制超时保护。
- 实机低速遥操作。

## 14. 测试计划

### 14.1 单元测试

- WiFi 命令帧编码和 XOR 校验。
- 命令半包、合包、错误帧头和错误 checksum。
- TCP Stream 数据半包与多包解析。
- Ping metadata/schema 解析。
- hostname、IP 和端口参数校验。

### 14.2 固件测试

- `pio run -e usb` 编译通过。
- `pio run -e wifi` 编译通过。
- `pio run -e serial` 编译通过。
- 三种环境不会同时启用。
- WiFi 密码错误时 GUI 状态正确且设备不死机。
- 主机断开后 M5 可接受新连接。
- Zero 和 Zero Mask 在 WiFi 下只执行一次。

### 14.3 ROS 2 测试

- USB/WiFi `/ker/joint_states` 名称和顺序一致。
- `/ker/error_mask` 一致。
- WiFi Ping 返回 metadata 和延迟。
- WiFi 断线后 0.25 秒内停止新控制输出。
- 重连后自动恢复数据。
- RViz 面板不会因 Ping 阻塞界面。

### 14.4 稳定性测试

- 同一局域网连续推流至少 30 分钟。
- 记录实际消息频率、平均延迟、最大延迟和重连次数。
- 路由器重启后自动恢复。
- M5 重启后主机自动恢复。
- WiFi 信号较弱时不产生错误角度跳变。

## 15. 验收标准

- [ ] USB 和 Serial 模式功能无回归。
- [ ] WiFi 固件可通过独立 PlatformIO 环境烧录。
- [ ] M5 和主机仅需处于同一局域网即可通过 hostname 或 IP 连接。
- [ ] WiFi Ping 能返回固件、硬件、更新时间和延迟。
- [ ] WiFi Stream 发布与 USB 相同的 ROS 2 话题和关节顺序。
- [ ] 网络断开时控制链路按超时策略停止更新。
- [ ] 网络恢复后无需重启 ROS 节点即可继续接收数据。
- [ ] RViz 面板可分别执行 USB Ping 和 WiFi Ping。
- [ ] M5 屏幕能显示 IP、端口和客户端状态。
- [ ] 两种固件版本可以通过 Ping 明确区分。
- [ ] WiFi 配置文件不会提交现场密码。

## 16. 预计改动文件清单

新增：

```text
ker/firmware/M5/include/StreamTypes.h
ker/firmware/M5/include/WiFiConfig.example.h
ker/firmware/M5/include/WiFiStream.h
ker/firmware/M5/src/WiFiStream.cpp
openflex_KER/WIFI_TRANSPORT_PLAN_CN.md
```

修改：

```text
ker/firmware/M5/.gitignore
ker/firmware/M5/platformio.ini
ker/firmware/M5/include/USBStream.h
ker/firmware/M5/include/SerialStream.h
ker/firmware/M5/include/GUIHandler.h
ker/firmware/M5/include/Meta.h
ker/firmware/M5/src/main.cpp
ker/firmware/M5/src/GUIHandler.cpp
openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_stream.py
openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_driver_node.py
openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml
openflex_KER/openflex_teleop_ker/launch/openflex_ker_teleop.launch.py
openflex_KER/openflex_teleop_ker/README_CN.md
openflex_KER/openflex_tool/openflex_ker_joint_panel/include/openflex_ker_joint_panel/joint_angle_panel.hpp
openflex_KER/openflex_tool/openflex_ker_joint_panel/src/joint_angle_panel.cpp
openflex_KER/openflex_tool/openflex_ker_joint_panel/README_CN.md
```

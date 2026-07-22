# OpenFlex KER WiFi 传输升级实施总结

日期：2026-07-21

本文记录本次按照 `WIFI_TRANSPORT_PLAN_CN.md` 完成的代码升级。升级目标是让
M5 固件在烧录时选择 USB、串口或 WiFi 传输，让机器人主机上的 ROS 2 驱动使用
同一套 KER schema 和角度处理链路，并在 RViz2 面板中分别检查 USB 和 WiFi。

## 1. 已实现的行为

- 新增 PlatformIO `wifi` 固件环境，M5 作为 TCP server，默认端口为 `19090`。
- WiFi 默认主机名为 `openarm-ker`，通过 mDNS 访问 `openarm-ker.local`；也可以在
  ROS YAML 中直接填写 M5 屏幕显示的 IPv4 地址。
- WiFi 数据帧仍使用官方 KER 的 stream 和 ping/schema 格式，ROS 上层不需要另一套
  关节映射或角度换算。
- WiFi 命令使用独立的 6 字节封装：`A5 43 cmd arg_hi arg_lo xor`。M5 收到后还原
  成官方 3 字节命令，因此 PING、STANDBY、STREAM、ZERO ALL、ZERO MASK 的语义不变。
- ROS 保留 `/ker/ping`，并新增 `/ker/ping_usb`、`/ker/ping_wifi`。
- RViz 面板新增 `USB Ping` 和 `WiFi Ping`，结果包含传输类型、端点、握手往返时间、
  HW、FW 和更新时间。
- USB、串口、WiFi 仍然是烧录时确定的单一传输方式，运行时不会自动切换；正常角度
  话题和 OpenFlex 控制桥的名称保持不变。

## 2. M5 固件文件

### `ker/firmware/M5/platformio.ini`

新增 `env:wifi`，使用 `USE_WIFI` 编译宏，默认仍为 `usb`。三个环境分别设置
`build_src_filter`，只编译当前传输实现，避免 WiFi 构建依赖 USB Vendor 实现，或串口
构建误编译不需要的网络代码。烧录命令为：

```bash
pio run -e usb --target upload
pio run -e serial --target upload
pio run -e wifi --target upload
```

### `ker/firmware/M5/include/StreamTypes.h`

提取公共的 `Type`、`FieldDef` 和 `CommandCallback` 定义。USB、串口和 WiFi 三种
stream 现在共享同一套字段描述，确保 schema 类型编号和字段布局一致。

### `ker/firmware/M5/include/WiFiConfig.example.h`

新增 WiFi 配置模板，包含 SSID、密码、mDNS 主机名和 TCP 端口。实际部署时复制为
`WiFiConfig.h` 并填写机器人局域网凭据；`.gitignore` 会忽略实际凭据文件。

### `ker/firmware/M5/include/WiFiStream.h`

新增 WiFi stream 类，提供与 `USBStream`/`SerialStream` 相同的 `add`、`set`、`send`、
`recv`、`onCommand` 和 `sendPingResponse` 接口。额外提供 IP、RSSI、WiFi 连接状态和
TCP client 状态，供 M5 屏幕显示。

### `ker/firmware/M5/src/WiFiStream.cpp`

实现 WiFi Station 模式、DHCP、mDNS、TCP server、单客户端管理、TCP_NODELAY、断线
重连和接收缓存。连接建立后通过已有 schema 发送角度数据；客户端断开后清空命令缓存，
避免半帧命令污染下一次连接。

命令解析严格检查 `A5 43` 头和 XOR 校验，合法帧才交给现有回调。因此 WiFi 不会直接
绕过原有 Zero 或状态机逻辑。当前设计只接受一个 TCP client，且没有 TLS 或认证，
必须部署在可信机器人局域网中。

### `ker/firmware/M5/include/USBStream.h`

移除重复的公共类型定义，改为包含 `StreamTypes.h`。USB 数据格式和行为未改变。

### `ker/firmware/M5/include/SerialStream.h`

同样改为使用 `StreamTypes.h`。串口传输格式和行为未改变。

### `ker/firmware/M5/src/main.cpp`

新增编译期检查，要求 `USE_USB`、`USE_SERIAL`、`USE_WIFI` 三者恰好定义一个；根据
编译宏实例化对应 stream，并在 setup 和 loop 中选择对应初始化、接收和发送路径。
WiFi 客户端断开且正在 STREAM 时自动回到 STANDBY，避免主机消失后 M5 继续处于
控制状态。

原有 PING、STANDBY、STREAM、Zero All 和 Zero Mask 回调保持不变。新增按传输方式的
固件版本字符串只用于 Ping 元数据，不改变角度、夹爪映射或标定参数。

### `ker/firmware/M5/include/Meta.h`

新增 USB、串口、WiFi 三个构建版本字符串，并把更新时间更新为 `2026-07-21`。
Ping 返回的硬件版本、固件版本和更新时间因此可以确认当前烧录的是哪种传输固件。

### `ker/firmware/M5/include/GUIHandler.h`

新增传输状态文本字段和 setter。

### `ker/firmware/M5/src/GUIHandler.cpp`

在 M5 屏幕左上角显示当前传输状态。WiFi 模式显示 IP 和客户端连接状态，USB 模式
显示 USB 是否连接，避免与右上角跳变检测控制重叠。

### `ker/firmware/M5/.gitignore`

忽略 `include/WiFiConfig.h`，避免 SSID 和密码进入版本库，同时保留可提交的示例配置。

### `ker/firmware/M5/README.md`

补充三种固件环境的构建/烧录命令、WiFi 配置复制步骤、mDNS 默认地址、端口和单客户
端限制，并更新目录结构说明。

本次没有修改 `ker/firmware/M5/include/Common.h` 中的编码器机械偏移、方向、夹爪参数
或 Zero 标定逻辑。

## 3. ROS 2 文件

### `openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_stream.py`

扩展 `KERStream` 支持 `wifi`。使用 TCP socket 连接 M5，连接超时和 socket 轮询超时
可配置；保留原有 USB/串口读取、schema 解析、stream packet 校验和字段动态解析。
新增 `endpoint` 属性用于 Ping 报告，并新增 `encode_wifi_command()` 生成 WiFi 命令帧。
TCP 暂时没有数据时按超时返回空字节，不会误判为错误；对端关闭连接则报告连接断开，
让驱动进入已有的重连流程。

### `openflex_KER/openflex_teleop_ker/openflex_teleop_ker/ker_driver_node.py`

新增参数：

- `wifi_host`，默认 `openarm-ker.local`
- `wifi_port`，默认 `19090`
- `wifi_connect_timeout_s`，默认 `3.0`
- `wifi_socket_timeout_s`，默认 `0.02`
- `ping_usb_service`，默认 `/ker/ping_usb`
- `ping_wifi_service`，默认 `/ker/ping_wifi`

`/ker/ping` 仍检查当前 `transport`。两个新增服务分别强制检查 USB 或 WiFi；当前
传输被检查时会暂时关闭并重新连接，完成 schema 握手后恢复 STREAM。检查另一种传输
时创建临时 stream，不抢占当前活动流。所有连接、关闭和 Ping 操作使用锁保护，避免
USB 设备或 TCP socket 被数据定时器与 Ping 同时操作。

Ping 响应使用现有 `std_srvs/srv/Trigger`，不引入新接口，消息中包含 transport、
endpoint、latency、HW、FW、updated 或具体失败原因。这样旧的 `/ker/ping` 调用方式
保持兼容。

### `openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml`

加入 WiFi 连接参数和两个新 Ping service 名称。原有 USB 默认值、串口参数、关节比例、
偏移、夹爪映射、话题名和安全参数均保持不变。

### `openflex_KER/openflex_teleop_ker/launch/openflex_ker_teleop.launch.py`

将 `transport` 的合法值从 `usb/serial` 扩展为 `usb/serial/wifi`。节点结构和桥接参数
不变，因此 WiFi 只替换底层设备连接，不改变 OpenFlex 控制链路。

### `openflex_KER/openflex_teleop_ker/setup.py`

版本更新为 `0.2.0`，描述更新为支持 USB、串口和 WiFi。

### `openflex_KER/openflex_teleop_ker/package.xml`

版本更新为 `0.2.0`，同步更新包描述；现有 Python ROS 依赖足以承载 TCP socket，
不需要新增第三方网络依赖。

### `openflex_KER/openflex_teleop_ker/test/test_ker_stream.py`

新增 4 组测试，覆盖 WiFi PING 命令帧补零、ZERO MASK 参数和 XOR 校验、非法 payload
长度拒绝，以及 USB/串口/WiFi endpoint 文本生成。

### `openflex_KER/openflex_teleop_ker/README_CN.md`

补充 WiFi 固件与 ROS launch 的匹配关系、mDNS/IP 配置、三个 Ping 服务和使用命令。
同时明确固件烧录模式必须与 launch 的 `transport` 一致。

## 4. RViz2 文件

### `openflex_KER/openflex_tool/openflex_ker_joint_panel/include/openflex_ker_joint_panel/joint_angle_panel.hpp`

将单一 Ping 控件改为 USB 和 WiFi 两个按钮、两个 ROS Trigger client 和共享 Ping
处理函数。关节订阅、左右臂表格和在线状态字段保持不变。

### `openflex_KER/openflex_tool/openflex_ker_joint_panel/src/joint_angle_panel.cpp`

实现两个 Ping 按钮的独立调用和状态显示。面板不会直接访问 USB 或 TCP，仍通过 ROS
驱动完成检查；Ping 状态与实时 `/ker/joint_states` 状态分开显示。原有识别
`ker_*` 和 `openarmx_*` 关节名、弧度/角度显示和话题切换功能不变。

### `openflex_KER/openflex_tool/openflex_ker_joint_panel/package.xml`

版本更新为 `0.2.0`。已有 `std_srvs` 依赖已经覆盖新增服务，不需要增加消息包。

### `openflex_KER/openflex_tool/openflex_ker_joint_panel/README_CN.md`

更新面板操作说明，说明两个 Ping 的服务名、返回信息、WiFi endpoint 配置位置，以及
Ping 状态和关节话题在线状态的区别。

`CMakeLists.txt` 本次无需修改，因为 Qt、rclcpp、sensor_msgs、std_srvs 依赖和插件
入口已经存在。

## 5. 验证结果与部署限制

已完成本地验证：

```text
8 tests passed
Python AST parse passed
YAML parse passed
ROS package XML parse passed
RViz plugin XML parse passed
```

当前开发环境没有安装 `pio`、`ros2` 和 `colcon`，因此本次无法在工作区内执行真正的
PlatformIO 编译、M5 烧录或 ROS 2/Qt 构建。实际机器人主机部署时建议按以下顺序操作：

1. 在 M5 固件目录复制并填写 `include/WiFiConfig.h`。
2. 只选择一个环境烧录，例如 `pio run -e wifi --target upload`。
3. 确认 M5 屏幕显示已取得 IP，并记录 IP 作为 mDNS 不可用时的备用地址。
4. 在机器人主机 ROS 工作空间构建两个包并 source 环境。
5. WiFi 固件使用 `transport:=wifi` 启动；USB 固件使用 `transport:=usb` 启动。
6. 先调用 `/ker/ping_wifi` 或面板 WiFi Ping，再观察 `/ker/joint_states`；确认角度
   和安全检查正常后再启动实际接管。

由于 WiFi TCP 当前没有认证和加密，不应把 `19090` 端口暴露到公网。传输升级没有
改变编码器标定、Zero 参考值、夹爪映射和 OpenFlex 控制安全阈值。

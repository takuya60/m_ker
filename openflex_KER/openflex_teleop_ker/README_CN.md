# openflex_teleop_ker 使用与集成手册

`openflex_teleop_ker` 是 OpenArm KER 与 OpenFlex/OpenArmX ROS 2 控制系统之间的适配包。
它读取 KER 的 16 路编码器数据，将固件角度转换为 ROS 2 关节目标，再通过带初始姿态
检查、渐进接管和目标超时保护的 bridge 输出到 ros2_control position controller。

本文覆盖信息通路、目录结构、ROS 接口、参数、构建、启动、RViz 联动、安全约束和排障。
固件机械零位和编码器配置见 [CALIBRATION_CN.md](CALIBRATION_CN.md)。

## 1. 功能范围

本包提供：

- USB、串口和 WiFi 三种 KER 传输。
- 固件自描述 schema 解析、checksum 校验和断线重连。
- 16 路源角度、错误位、设备信息发布。
- 右臂、右夹爪、左臂、左夹爪目标映射。
- OpenArmX forward position controller 输出。
- 接管前最大关节差检查和 3 秒线性插值。
- 输入目标超时停止发布。
- 低频、变化触发的 J1-J7 调试日志。
- OpenArmX fake hardware 与 RViz 一键仿真。

本包不负责 MoveIt 规划、碰撞检测、速度限制、CAN 硬件驱动或固件烧录。目标超时后
controller 通常保持最后位置，本包不会自动回零。

## 2. 信息通路

```text
KER 编码器 ID 1-16
  -> M5 固件：方向、Zero、机械偏移、错误检测
  -> USB / serial / WiFi
  -> KERStream：PING schema、STREAM 解包
  -> openflex_ker_driver
       -> /ker/joint_states
       -> /ker/error_mask
       -> /ker/metadata
       -> /ker/right_arm/joint_command
       -> /ker/left_arm/joint_command
  -> openflex_ker_arm_bridge
       -> 初始姿态检查
       -> 渐进接管
       -> 目标超时保护
  -> /right_forward_position_controller/commands
     /left_forward_position_controller/commands
  -> ros2_control fake/real hardware
  -> joint_state_broadcaster -> /joint_states
  -> robot_state_publisher -> /tf
  -> RViz RobotModel
```

RViz 不直接消费 KER 命令。RViz 模型运动依赖：

```text
controller command -> hardware state -> /joint_states -> TF -> RViz
```

因此 KER 目标有变化但 RViz 不动时，必须继续检查 controller、
`joint_state_broadcaster` 和 `robot_state_publisher`。

## 3. 通道与关节映射

| KER 通道 | 固件索引 | KER 源名称 | OpenArmX 目标名称 |
|---|---:|---|---|
| 1-7 | 0-6 | `ker_right_joint1...7` | `openarmx_right_joint1...7` |
| 8 | 7 | `ker_right_gripper` | `openarmx_right_finger_joint1` |
| 9-15 | 8-14 | `ker_left_joint1...7` | `openarmx_left_joint1...7` |
| 16 | 15 | `ker_left_gripper` | `openarmx_left_finger_joint1` |

手臂转换公式：

```text
target_rad = radians(firmware_angle_deg) * joint_scale + joint_offset
```

`joint_scales` 和 `joint_offsets` 各有 14 项，顺序为右 J1-J7、左 J1-J7。

夹爪映射：

- KER 右夹爪：`0 deg` 打开，`-60 deg` 闭合。
- KER 左夹爪：`0 deg` 打开，`+60 deg` 闭合。
- OpenArmX：`0.044 m` 打开，`0.0 m` 闭合。

## 4. 文件和脚本

```text
openflex_teleop_ker/
├── openflex_teleop_ker/
│   ├── ker_stream.py
│   ├── pose_processor.py
│   ├── ker_driver_node.py
│   ├── arm_bridge_node.py
│   └── __init__.py
├── config/openflex_ker.yaml
├── launch/openflex_ker_teleop.launch.py
├── launch/openflex_ker_rviz_sim.launch.py
├── test/test_ker_stream.py
├── test/test_pose_processor.py
├── setup.py
├── setup.cfg
├── package.xml
├── README_CN.md
└── CALIBRATION_CN.md
```

| 文件 | 作用 |
|---|---|
| `ker_stream.py` | 打开传输端点、获取 schema、解析数据包、发送 STANDBY/STREAM。 |
| `pose_processor.py` | 度转弧度、scale/offset、夹爪映射、可选 Hampel 滤波。 |
| `ker_driver_node.py` | 连接和重连 KER，发布源状态、错误位、元数据及左右目标。 |
| `arm_bridge_node.py` | 按关节名读取机器人状态，检查初始差值，输出 controller 命令。 |
| `openflex_ker.yaml` | driver 与 bridge 的默认参数。 |
| `openflex_ker_teleop.launch.py` | 已有机器人 bringup 时启动 driver 和 bridge。 |
| `openflex_ker_rviz_sim.launch.py` | 一键启动 fake hardware、RViz、USB driver 和右臂仿真 bridge。 |
| `setup.py` | 安装 Python 包、配置、launch 并注册 console scripts。 |
| `package.xml` | ROS 包元数据和运行依赖。 |
| `test/` | 协议帧和角度映射单测。 |

## 5. 核心组件

### 5.1 `KERStream`

连接过程：打开 USB/串口/TCP，发送 `STANDBY`，周期发送 `PING` 获取固件 schema，启动
后台接收线程，然后由 driver 发送 `STREAM`。数据帧通过帧头和 XOR checksum 校验。

接收队列深度为 2。消费端变慢时丢弃旧包、保留最新状态，避免遥操延迟累积。关闭连接时
尽力发送 `STANDBY`。

默认端点：

- USB：`0x303a:0x4002`
- serial：`/dev/ttyACM0 @ 2000000`
- WiFi：`openarm-ker.local:19090`

### 5.2 `KerPoseProcessor`

固件必须提供 16 项 `angles`。处理器可选用 5 样本 Hampel 窗口抑制孤立跳点，然后发布
源弧度数据，并计算两侧手臂及夹爪目标。滤波不能用于掩盖机械松动、错误 ID 或 360 度
回绕。

### 5.3 `openflex_ker_driver`

driver 按 `publish_rate_hz` 消费最新数据，断开后按 `reconnect_interval_s` 重连。
每帧发布 `/ker/joint_states` 和 `/ker/error_mask`。默认任一错误位非零时停止左右目标发布。

错误位定义：bit 0 对应通道 1，bit 15 对应通道 16。

### 5.4 `openflex_ker_arm_bridge`

bridge 按 OpenArmX 关节名读取 `/joint_states`，而不是依赖数组顺序。首次目标到达后比较
机器人当前 J1-J7 与 KER 目标；任一差值超过 `max_joint_diff_rad` 就拒绝该侧接管，直到
节点重启。

检查通过后，bridge 在 `interpolation_duration` 内从当前姿态线性过渡到首次目标，之后按
`command_rate_hz` 发布最新目标。超过 `target_timeout_s` 没有新目标时停止发布。

## 6. ROS 2 接口

### 6.1 Driver 发布

| 话题 | 类型 | 内容 |
|---|---|---|
| `/ker/joint_states` | `sensor_msgs/msg/JointState` | KER 16 路源角度，单位 rad。 |
| `/ker/error_mask` | `std_msgs/msg/UInt16` | 16 位传感器故障掩码。 |
| `/ker/metadata` | `std_msgs/msg/String` | JSON 形式端点和 FW/HW 信息。 |
| `/ker/right_arm/joint_command` | `sensor_msgs/msg/JointState` | 右 J1-J7 和夹爪目标。 |
| `/ker/left_arm/joint_command` | `sensor_msgs/msg/JointState` | 左 J1-J7 和夹爪目标。 |

### 6.2 Bridge 接口

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/ker/right_arm/joint_command` | `sensor_msgs/msg/JointState` |
| 输入 | `/ker/left_arm/joint_command` | `sensor_msgs/msg/JointState` |
| 输入 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 输出 | `/right_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| 输出 | `/left_forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |

每侧 controller 消息固定为 8 项：J1-J7、夹爪。

### 6.3 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `/ker/ping` | `std_srvs/srv/Trigger` | 检查当前传输。 |
| `/ker/ping_usb` | `std_srvs/srv/Trigger` | 强制检查 USB。 |
| `/ker/ping_wifi` | `std_srvs/srv/Trigger` | 强制检查 WiFi。 |

## 7. 参数

### 7.1 Driver 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `transport` | `usb` | `usb`、`serial` 或 `wifi`。 |
| `serial_port` | `/dev/ttyACM0` | 串口设备。 |
| `baud_rate` | `2000000` | 串口波特率。 |
| `usb_vid` | `12346` | `0x303A`。 |
| `usb_pid` | `16386` | `0x4002`。 |
| `wifi_host` | `openarm-ker.local` | M5 主机名或 IPv4。 |
| `wifi_port` | `19090` | TCP 端口。 |
| `wifi_connect_timeout_s` | `3.0` | 建连超时。 |
| `wifi_socket_timeout_s` | `0.02` | 接收超时。 |
| `publish_rate_hz` | `100.0` | driver 处理频率。 |
| `reconnect_interval_s` | `2.0` | 重连间隔。 |
| `use_hampel_filter` | `false` | 是否启用异常点滤波。 |
| `drop_command_on_sensor_error` | `true` | 任一错误位时停止目标。 |
| `gripper_min_position` | `0.0` | 夹爪闭合位置。 |
| `gripper_max_position` | `0.044` | 夹爪打开位置。 |
| `joint_scales` | 14 个 `1.0` | 手臂比例和方向。 |
| `joint_offsets` | 14 个 `0.0` | 手臂弧度偏移。 |

YAML 还可以配置源话题、错误话题、左右目标话题和三个 Ping 服务名。

方向和零位应优先在固件中修正，不应同时改固件 `invert` 和 ROS scale。

### 7.2 Bridge 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `joint_states_topic` | `/joint_states` | 机器人当前状态。 |
| `left_controller_topic` | `/left_forward_position_controller/commands` | 左输出。 |
| `right_controller_topic` | `/right_forward_position_controller/commands` | 右输出。 |
| `enable_safety_check` | `true` | 初始姿态检查。 |
| `max_joint_diff_rad` | `0.873` | 最大初始差，约 50 度。 |
| `interpolation_duration` | `3.0` | 接管插值时间。 |
| `command_rate_hz` | `50.0` | 命令频率。 |
| `target_timeout_s` | `0.25` | 目标超时。 |
| `log_joint_changes` | `false` | 是否打印变化日志。 |
| `joint_log_rate_hz` | `1.0` | 日志最高频率。 |
| `joint_log_min_change_rad` | `0.005` | 最小变化阈值。 |

仿真 launch 默认启用低频日志：

```text
[right] KER target [rad]: J1=+0.035, J2=+0.000, ... J7=+0.000
```

## 8. 安装与构建

```bash
sudo apt update
sudo apt install -y \
  python3-numpy python3-serial python3-usb \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-ros2controlcli \
  ros-humble-forward-command-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro
```

USB udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="303a", ATTR{idProduct}=="4002", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-openflex-ker.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插 M5 后检查：

```bash
lsusb -d 303a:4002
python3 -c "import usb; print(usb.__version__)"
```

当前仓库克隆在工作空间 `src/m_ker`，构建命令不依赖源码的具体深度：

```bash
cd /home/openflex/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  openarmx_description openarmx_bringup openflex_teleop_ker
source install/setup.bash
```

确认实际包来源：

```bash
colcon list | grep '^openflex_teleop_ker'
ros2 pkg prefix openflex_teleop_ker
```

## 9. 启动

### 9.1 RViz fake hardware 一键仿真

前提：OpenArmX 实机断电或 CAN 断开；KER J1-J7 在参考零位；没有其他 OpenArmX、
MoveIt、controller manager 或 KER launch。

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

使用 WiFi 固件时：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py \
  transport:=wifi \
  wifi_host:=192.168.3.114 \
  wifi_port:=19090
```

当前 launch 文件顶部的默认值为：

```python
DEFAULT_TRANSPORT = 'wifi'
DEFAULT_WIFI_HOST = '192.168.3.114'
DEFAULT_WIFI_PORT = '19090'
```

因此当前设备可以直接启动：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

命令行参数仍可临时覆盖这些默认值，例如切回 USB：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py transport:=usb
```

独立测试脚本和 ROS driver 不能同时连接 M5。启动该 launch 前，先停止
`firmware/test/wifi_stream_test.py`。

该 launch 固定：

- 包含 `openarmx_bringup/openarmx.bimanual.launch.py`。
- 使用 fake hardware 和 forward position controller。
- 关闭 forward effort。
- KER 默认使用 USB，也可通过 launch 参数选择 serial 或 WiFi。
- 仅仿真覆盖 `drop_command_on_sensor_error=false`。
- 左臂输出到 `/ker/sim/disabled_left_controller/commands`。
- 保持初始姿态安全检查。
- bridge 延迟 4 秒启动。
- 以最高 1 Hz 打印变化的关节目标。

只有通道 1-7 时 `error_mask=65408` 是预期值，但此配置禁止用于实机。

成功日志：

```text
KER connected via usb
joint_state_broadcaster ... activate successful
right_forward_position_controller ... activate successful
[right] safe takeover started
```

### 9.2 已有机器人 bringup

外部系统必须先启动 `/joint_states` 和左右 forward position controller：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py
```

串口和 WiFi：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py transport:=serial
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py transport:=wifi
```

通用 launch 不启动机器人或 RViz。实机使用前必须完成全部通道标定，并保持
`drop_command_on_sensor_error=true`。

### 9.3 只启动 driver

```bash
ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/openflex/openflex_all/openflex_ws/src/m_ker/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml
```

通道 1-7 的 fake hardware 验证才允许追加：

```text
-p drop_command_on_sensor_error:=false
```

### 9.4 KER driver 与右臂仿真 bridge 一起启动

推荐用于“终端 1 启动 OpenArmX + RViz，终端 2 启动 KER”的调试方式：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py
```

该 launch 不启动机器人、controller 或 RViz，只启动：

- `openflex_ker_driver`
- `openflex_ker_arm_bridge`

默认使用 `wifi / 192.168.3.114 / 19090`，仅在 fake hardware 调试中关闭全局传感器错误
丢弃，并将左臂 controller 输出隔离到废弃话题。初始姿态安全检查保持开启。

仍可覆盖默认配置：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py \
  wifi_host:=192.168.3.120
```

### 9.5 单独启动 bridge

```bash
ros2 run openflex_teleop_ker ker_arm_bridge_node --ros-args \
  --params-file /home/openflex/openflex_all/openflex_ws/src/m_ker/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml
```

右臂-only fake 测试还要覆盖左输出：

```text
-p left_controller_topic:=/ker/sim/disabled_left_controller/commands
```

## 10. 与其他包联动

### 10.1 `openarmx_bringup` 与 RViz

OpenArmX bringup 提供 ros2_control、fake/real hardware、controller、
`joint_state_broadcaster`、`robot_state_publisher` 和 RViz。右 controller 的 8 项顺序必须为：

```text
openarmx_right_joint1 ... openarmx_right_joint7,
openarmx_right_finger_joint1
```

### 10.2 自定义 ros2_control 包

外部包必须：

1. 发布包含 OpenArmX J1-J7 名称的 `JointState`。
2. 提供接收 `Float64MultiArray` 的位置 controller。
3. 每侧接受 J1-J7、夹爪共 8 项。
4. 将执行后的状态反馈到 `/joint_states`。

话题可以在 YAML 中修改：

```yaml
openflex_ker_arm_bridge:
  ros__parameters:
    joint_states_topic: /my_robot/joint_states
    right_controller_topic: /my_robot/right_controller/commands
    left_controller_topic: /my_robot/left_controller/commands
```

目标关节名目前是代码常量。其他机器人关节名称不同时，需要修改
`pose_processor.py`，或增加 JointState 名称适配节点。

### 10.3 MoveIt

bridge 输出 `Float64MultiArray`，不能直接连接 trajectory controller。同一关节的 trajectory
controller 和 forward position controller 不能同时 claim position interface。不要同时启动
MoveIt demo 和 KER RViz sim。

## 11. 运行检查

KER 通信：

```bash
ros2 service call /ker/ping std_srvs/srv/Trigger
ros2 topic echo /ker/metadata --once
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/error_mask --once
```

目标链：

```bash
ros2 topic echo /ker/right_arm/joint_command --once
ros2 topic echo /right_forward_position_controller/commands --once
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

应至少有：

```text
joint_state_broadcaster active
left_forward_position_controller active
right_forward_position_controller active
```

逐轴验收时每次只移动约 5 度，检查 KER 目标、controller command、`/joint_states` 和
RViz 中的同一关节依次变化。

## 12. 安全机制与限制

1. 默认任一固件错误位停止目标更新。
2. 缺少一侧任一目标名称时忽略整条消息。
3. 初始差超过约 50 度时拒绝接管。
4. 默认 3 秒渐进接管。
5. 目标超过 0.25 秒未更新时停止 bridge 发布。
6. driver 关闭时向 KER 发送 STANDBY。

限制：错误保护目前是全局的；bridge 没有正式 `right_arm_only` 参数；初始检查不比较夹爪；
接管后不持续做姿态差检查；没有速度、碰撞和主动回零保护。

## 13. 常见问题

### `No module named 'usb'`

```bash
sudo apt install -y python3-usb
```

### `Access denied (insufficient permissions)`

安装 udev 规则并拔插 M5。不要用 `sudo ros2 run`。

### KER 状态有数据但目标没有数据

检查 `/ker/error_mask`。默认错误保护会停止左右目标。

### `[right] takeover rejected`

停止 bridge，让 KER 与机器人回到同一参考姿态后重启。不要关闭安全检查绕过。

### controller command 有变化但 RViz 不动

检查 controller 是否 active、`/joint_states` 是否变化、robot_state_publisher 和 TF 是否正常。

### controller 加载随机失败

检查重复实例：

```bash
ros2 node list | sort
ps -ef | grep -E 'ros2 launch|ros2_control_node|rviz2'
```

出现两个 `/controller_manager`、两个 `/robot_state_publisher` 或两个 RViz 时，先停止所有旧
launch，再只启动一套。同名 controller manager 会导致 spawner 请求被错误实例响应。

### 通道 8-16 ERR

只有 1-7 安装时对应 `error_mask=65408`。该状态只允许右臂 fake hardware 测试。

## 14. 停止和测试

停止顺序：bridge、driver、机器人 bringup、RViz、M5。一键 launch 使用一次 `Ctrl+C`，并用
`ros2 node list` 确认无残留。

测试：

```bash
cd /home/openflex/openflex_all/openflex_ws/src
PYTHONPATH="$PWD/m_ker/openflex_KER/openflex_teleop_ker" \
  python3 -m pytest -q m_ker/openflex_KER/openflex_teleop_ker/test

python3 -m py_compile \
  m_ker/openflex_KER/openflex_teleop_ker/openflex_teleop_ker/*.py \
  m_ker/openflex_KER/openflex_teleop_ker/launch/*.launch.py
```

## 15. 仓库内嵌套工作空间

仓库还保留了 `openflex_KER/ros2_ws/src`、`build`、`install` 和 `log`，这是另一台主机曾经
使用过的独立 colcon 工作空间快照，其中 `src/openflex_teleop_ker` 是旧副本。

顶层工作空间递归扫描时，如果同时发现主包和这个旧副本，会报同名包或不确定使用哪一份。
因此仓库中的 `openflex_KER/ros2_ws/COLCON_IGNORE` 用于让当前顶层工作空间忽略整个嵌套
工作空间。实际维护和构建的唯一源码是：

```text
m_ker/openflex_KER/openflex_teleop_ker
```

## 16. 相关文档

- [CALIBRATION_CN.md](CALIBRATION_CN.md)：机械零位和固件标定。
- `openflex_KER/KER_OPENARMX_RVIZ_SIM_CALIBRATION_GUIDE_CN.md`：1-7 通道 RViz 验收。
- `openflex_KER/WIFI_TRANSPORT_IMPLEMENTATION_SUMMARY_CN.md`：WiFi 实现说明。

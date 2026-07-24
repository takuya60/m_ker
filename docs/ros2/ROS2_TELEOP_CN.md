# KER ROS 2 遥操使用手册

本文说明 `openflex_teleop_ker` 如何把 KER 的角度数据接入 ROS 2、OpenArmX 和 RViz。

## 1. 功能简介

ROS 2 部分包含两个主要节点：

- `ker_driver_node`：连接 KER，解析 USB/serial/WiFi 数据，执行滤波和关节映射。
- `ker_arm_bridge_node`：读取 KER 目标，执行速度受限插值，并发布给 ros2_control。

## 2. 数据通路

```text
KER M5
  -> ker_driver_node
  -> /ker/joint_states
  -> 滤波、joint_scales/joint_offsets 映射
  -> /ker/right_arm/joint_command
   /ker/left_arm/joint_command
  -> ker_arm_bridge_node
  -> forward position controller
  -> /joint_states
  -> robot_state_publisher
  -> RViz RobotModel
```

关节目标顺序如下：

```text
右 J1-J7、右夹爪、左 J1-J7、左夹爪
```

## 3. 节点与接口

### 3.1 `ker_driver_node`

主要职责：

- 连接 KER 并自动重连。
- 读取自描述 schema 和数据包。
- 发布原始角度、错误位和设备信息。
- 执行 Hampel/一阶低通滤波。
- 将固件角度转换为 OpenArmX 关节目标。

主要发布话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/ker/joint_states` | `sensor_msgs/msg/JointState` | KER 角度，单位为弧度。 |
| `/ker/error_mask` | `std_msgs/msg/UInt16` | 16 路错误位。 |
| `/ker/metadata` | `std_msgs/msg/String` | 固件、硬件和传输信息。 |
| `/ker/right_arm/joint_command` | `sensor_msgs/msg/JointState` | 右臂目标。 |
| `/ker/left_arm/joint_command` | `sensor_msgs/msg/JointState` | 左臂目标。 |

服务：

```text
/ker/ping
/ker/ping_usb
/ker/ping_wifi
```

### 3.2 `ker_arm_bridge_node`

主要职责：

- 按关节名称读取 `/joint_states`，不依赖数组顺序。
- 从当前机器人姿态开始接管。
- 按最大关节/夹爪速度缓慢追踪最新目标。
- 超过目标超时时间后停止发布。

默认输出：

```text
/right_forward_position_controller/commands
/left_forward_position_controller/commands
```

## 4. 参数配置

默认配置文件：

```text
openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml
```

常用 driver 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `transport` | `usb` | `usb`、`serial` 或 `wifi`。 |
| `serial_port` | `/dev/ttyACM0` | serial 模式设备。 |
| `baud_rate` | `2000000` | serial 模式波特率。 |
| `wifi_host` | `openarm-ker.local` | WiFi 主机名或 IP。 |
| `wifi_port` | `19090` | WiFi TCP 端口。 |
| `publish_rate_hz` | `100.0` | driver 发布频率上限。 |
| `use_low_pass_filter` | `true` | 是否启用一阶低通。 |
| `low_pass_alpha` | `0.2` | 越小越平滑，延迟越大。 |
| `use_hampel_filter` | `false` | 是否抑制孤立异常点。 |
| `drop_command_on_sensor_error` | `true` | 有错误位时停止目标发布。 |

常用 bridge 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `max_joint_velocity_rad_s` | `3.0` | 关节最大跟随速度。 |
| `max_gripper_velocity_m_s` | `0.3` | 夹爪最大跟随速度；Panel 可配置范围为 `0.001-1.0 m/s`。 |
| `command_rate_hz` | `50.0` | 控制器命令发布频率。 |
| `target_timeout_s` | `0.25` | 无新目标后停止发布的时间。 |
| `log_joint_changes` | `false` | 是否打印低频关节变化日志。 |

`joint_scales` 和 `joint_offsets` 顺序为右 J1-J7、左 J1-J7。当前左右 J7 的
`joint_scale` 为 `-1.0`，用于匹配 KER 和 OpenArmX 的方向定义。

运动参数也可以在 RViz Joint Panel 的“运动参数”页面在线修改。在线修改只作用于当前
节点运行周期；节点重启后恢复 YAML 中的值。

## 5. 构建工作空间

```bash
cd /home/openflex/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  openflex_teleop_ker openflex_ker_joint_panel
source install/setup.bash
```

每个新终端都需要重新 source：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
```

## 6. 启动方式

### 6.1 只启动 driver

USB：

```bash
ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/openflex/openflex_all/openflex_ws/src/m_ker/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=usb
```

WiFi：

```bash
ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/openflex/openflex_all/openflex_ws/src/m_ker/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=wifi \
  -p wifi_host:=192.168.3.114 \
  -p wifi_port:=19090
```

### 6.2 启动 driver 和 bridge

真实 OpenFlex 已经单独启动 bringup 时：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py
```

右臂仿真遥操（driver + bridge，不启动 RViz）：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py
```

覆盖滤波、速度和目标超时参数：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py \
  drop_command_on_sensor_error:=false \
  use_low_pass_filter:=true \
  low_pass_alpha:=0.2 \
  max_joint_velocity_rad_s:=3.0 \
  max_gripper_velocity_m_s:=0.3 \
  target_timeout_s:=0.25
```

`ros2 launch` 只能覆盖 launch 文件通过 `DeclareLaunchArgument` 声明的参数。查看当前
支持的全部参数：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py --show-args
```

该 launch 默认使用：

```text
transport=wifi
wifi_host=192.168.3.112
wifi_port=19090
```

### 6.3 单独启动双臂 RViz

只启动 OpenArmX 双臂 fake hardware、forward position controller 和 RViz，不连接 KER：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller \
  enable_forward_effort:=false
```

需要 KER 遥操时，在另一个终端启动
`openflex_ker_right_sim_teleop.launch.py`。

### 6.4 一键启动 RViz 仿真

该 launch 会启动 OpenArmX fake hardware、driver、bridge 和 RViz：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

覆盖 WiFi 参数：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py \
  transport:=wifi \
  wifi_host:=192.168.3.114 \
  wifi_port:=19090
```

### 6.5 真机双臂遥操

真机启动前确认 CAN 接口正确、急停可用、已了解当前缺失的编码器通道，并让 KER 与
真机处于接近的初始姿态。

终端 1 启动真机、双臂 controller 和 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  enable_forward_effort:=false
```

终端 2 启动 KER driver 和双臂 bridge：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py \
  transport:=wifi \
  wifi_host:=openarm-ker.local \
  wifi_port:=19090 \
  drop_command_on_sensor_error:=false \
  max_joint_velocity_rad_s:=3.0 \
  max_gripper_velocity_m_s:=0.3 \
  target_timeout_s:=0.25
```

USB 固件把第二条命令改为 `transport:=usb`。设置
`drop_command_on_sensor_error:=false` 后，`/ker/error_mask` 仍会报告缺失通道，但 driver
会继续发布关节目标。缺失通道会使用固件当前提供的值，因此启动前需要确认对应真机关节
不会发生非预期运动。

## 7. RViz Panel 使用

启动 RViz 后添加 KER Panel：

```text
Panels -> Add New Panel -> openflex_ker_joint_panel/JointAnglePanel
```

Panel 包含两个页面：

- `关节角度`：显示左右臂 J1-J7、夹爪角度、话题频率、延迟和 Ping 结果。
- `运动参数`：读取和修改低通/Hampel 滤波、关节/夹爪最大速度及目标超时。

关节角度页面默认订阅 `/ker/joint_states`，也可以切换为 `/joint_states` 查看机器人反馈。

运动参数页面使用方法：

1. 确认 `ker_driver_node` 和 `ker_arm_bridge_node` 已启动。
2. 点击 `读取当前参数`。
3. 修改滤波或速度参数。
4. 点击 `应用参数`。

参数会立即作用于当前节点，但不会写回 YAML；节点重启后恢复配置文件中的默认值。

## 8. 检查与排障

查看节点：

```bash
ros2 node list
```

检查数据和频率：

```bash
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/joint_states --once
ros2 topic echo /ker/error_mask --once
ros2 topic echo /ker/right_arm/joint_command --once
```

检查控制器

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

常见问题：

| 现象 | 检查项 |
|---|---|
| `No module named usb` | 安装 `python3-usb`。 |
| USB Access denied | 配置 udev 规则或检查设备权限。 |
| WiFi 无法连接 | 检查 IP、端口、局域网和单客户端限制。 |
| `/ker/joint_states` 无数据 | 检查 driver 连接日志和 M5 是否处于 STREAM。 |
| `/joint_states` 无数据 | 检查 robot bringup 和 joint state broadcaster。 |
| bridge 不输出 | 检查目标话题、控制器名称和 `target_timeout_s`。 |
| 关节方向相反 | 检查固件方向与 `joint_scales`，避免重复反向。 |

`/ker/error_mask` 的 bit 0 对应通道 1，bit 15 对应通道 16。完整 16 路正常设备通常应为
`0`。

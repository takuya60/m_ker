# KER 1-7 通道遥操 OpenArmX RViz 仿真与标定指导

日期：2026-07-22

## 1. 本阶段目标

使用我们已经完成的 `openflex_teleop_ker` ROS 2 节点，让只有编码器通道 1-7 的 KER
驱动 OpenArmX 右臂 fake hardware，并在 RViz 中观察 J1-J7 实时运动。

本阶段范围：

- KER 通道 1-7，对应右臂 J1-J7。
- M5 使用 USB 固件。
- OpenArmX 使用 ROS 2 fake hardware。
- 使用 `right_forward_position_controller`。
- RViz 显示 OpenArmX 双臂模型，但只验收右臂 J1-J7。
- 标定修改集中在 M5 固件 `Common.h` 和 M5 Zero 数据。

本阶段不包含：

- 不连接或驱动 OpenArmX 实机电机。
- 不使用 CAN0/CAN1。
- 不遥操左臂。
- 不遥操右夹爪，因为通道 8 未安装。
- 不使用 Dora。
- 不测试 WiFi，先完成 USB RViz 仿真。

## 2. 总体数据链路

```text
KER 编码器 ID 1-7
  -> M5 USB 固件
  -> KERStream(usb)
  -> openflex_ker_driver
       -> /ker/joint_states
       -> /ker/error_mask
       -> /ker/right_arm/joint_command
  -> openflex_ker_arm_bridge
  -> /right_forward_position_controller/commands
  -> OpenArmX ros2_control fake hardware
  -> /joint_states
  -> robot_state_publisher
  -> RViz RobotModel
```

左臂目标仍会由当前驱动计算，但 bridge 的左臂输出会重映射到一个没有订阅者的废弃话题：

```text
/ker/sim/disabled_left_controller/commands
```

因此左臂 fake controller 不会收到 KER 指令。

## 3. 只有 1-7 通道时的特殊处理

### 3.1 错误位

通道 1-7 正常、8-16 缺失时：

```text
errors[0:7]   = False
errors[7:16]  = True
error_mask    = 0xFF80
十进制        = 65408
```

当前驱动默认参数：

```yaml
drop_command_on_sensor_error: true
```

它会在任意通道错误时禁止左右臂目标，因此默认配置下 RViz 不会跟随。纯 fake hardware
仿真时需要临时覆盖为：

```text
drop_command_on_sensor_error:=false
```

该覆盖只能用于本文的 RViz fake hardware 测试，绝不能用于 OpenArmX 实机。

### 3.2 缺失的右夹爪

通道 8 缺失时，固件保持的角度通常为 `0 deg`。当前映射会把它转换为 OpenArmX
右夹爪的 `0.044`，即打开位置。`right_forward_position_controller` 是 8-DOF，因此
bridge 仍会发送 8 个数值：

```text
[right J1 ... right J7, right gripper]
```

本阶段只验收前 7 项。夹爪保持打开只是为了满足 8-DOF fake controller 的消息长度，
不代表夹爪已经标定。

### 3.3 缺失的左臂

不要把 `left_controller_topic` 指向真实的
`/left_forward_position_controller/commands`。本文启动 bridge 时覆盖为：

```text
left_controller_topic:=/ker/sim/disabled_left_controller/commands
```

这样即使驱动生成左臂无效目标，也不会进入左臂 fake controller。

## 4. 安全要求

1. OpenArmX 实机断电，或至少断开 CAN 和电机电源。
2. 启动参数必须包含 `use_fake_hardware:=true`。
3. 控制器必须使用 `forward_position_controller`，与 KER bridge 的话题类型一致。
4. 不运行真实硬件 launch。
5. 不关闭 bridge 的初始角差检查。
6. `drop_command_on_sensor_error=false` 只存在于驱动命令行，不写回实机 YAML 默认值。
7. 左臂 controller 输出必须重映射到废弃话题。
8. M5 当前不能执行 Zero All，只能选择通道 1-7 后执行 Confirm Zero。

## 5. 准备统一 ROS 2 工作空间

建立独立仿真工作空间：

```bash
mkdir -p /home/alientek/openflex/openarmx_ker_sim_ws/src
cd /home/alientek/openflex/openarmx_ker_sim_ws/src
```

链接当前代码：

```bash
ln -s /home/alientek/openflex/openarmx_ros2/openarmx_bringup .
ln -s /home/alientek/openflex/openflex_KER/openflex_teleop_ker .
```

OpenArmX 模型仓库当前目录中不存在，需要获取：

```bash
git clone https://github.com/OpenFleX-Wheeled-Humanoid/openarmx_description.git
```

安装 ROS 依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-forward-command-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  python3-usb \
  python3-serial
```

构建：

```bash
cd /home/alientek/openflex/openarmx_ker_sim_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select \
  openarmx_description \
  openarmx_bringup \
  openflex_teleop_ker
source install/setup.zsh
```

检查：

```bash
ros2 pkg prefix openarmx_description
ros2 pkg prefix openarmx_bringup
ros2 pkg prefix openflex_teleop_ker
```

三个命令都应返回当前 `openarmx_ker_sim_ws/install` 下的路径。

## 6. 先单独验证 OpenArmX fake hardware

在连接 KER 之前，先证明 OpenArmX 仿真和 controller 正常。

### 6.1 启动

```bash
cd /home/alientek/openflex/openarmx_ker_sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller \
  enable_forward_effort:=false
```

启动日志必须表明使用 fake hardware。不能出现 CAN 通信、真实电机状态或
`openarmx_hardware` 实机启动信息。

### 6.2 检查 controller

另开终端：

```bash
cd /home/alientek/openflex/openarmx_ker_sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 control list_controllers
```

应至少看到：

```text
joint_state_broadcaster              active
left_forward_position_controller    active
right_forward_position_controller   active
```

检查右臂 controller 关节顺序：

```bash
ros2 param get /right_forward_position_controller joints
```

预期为右臂 J1-J7 加右夹爪，共 8 项。

### 6.3 手动发一条小角度命令

```bash
ros2 topic pub --once \
  /right_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.044]}"
```

预期：

- RViz 右臂 J1 小幅运动。
- 左臂不动。
- `/joint_states` 中 `openarmx_right_joint1` 接近 `0.10 rad`。

恢复全零：

```bash
ros2 topic pub --once \
  /right_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.044]}"
```

如果手动命令都不能驱动 RViz，不进入 KER 调试，应先处理 OpenArmX fake controller。

## 7. M5 固件标定

### 7.1 标定坐标目标

OpenArmX fake hardware 的初始配置在
`openarmx_bimanual_moveit_config/config/initial_positions.yaml` 中将左右 J1-J7 定义为
`0 rad`。本阶段使用相同约定：

```text
KER 右臂参考姿态 -> M5 angles[0:7] = 0 deg
                    -> ROS 右臂目标 = 0 rad
                    -> OpenArmX RViz 右臂零姿态
```

### 7.2 固件修改位置

文件：

```text
/home/alientek/openflex/ker/firmware/M5/include/Common.h
```

修改 `ENCODER_CONFIG` 中通道 1-7 的第 4 个字段
`mech_joint_offset`。保留已确认的 `invert`、`mech_min`、`mech_max` 和
`skip_jump_detect`：

`ENCODER_CONFIG` 数组的前 7 项应调整为：

```cpp
// invert   mech_min  mech_max  mech_joint_offset  skip_jd
{false,     -80.0f,   160.0f,   0.0f,              false}, // ch1 right J1
{true,      -10.0f,   160.0f,   0.0f,              false}, // ch2 right J2
{false,     -90.0f,    90.0f,   0.0f,              false}, // ch3 right J3
{true,        0.0f,   140.0f,   0.0f,              false}, // ch4 right J4
{false,     -90.0f,    90.0f,   0.0f,              false}, // ch5 right J5
{true,      -45.0f,    45.0f,   0.0f,              false}, // ch6 right J6
{true,      -90.0f,    90.0f,   0.0f,              false}, // ch7 right J7
```

只替换数组中的前 7 行；原有通道 8-16 的 9 行配置必须继续保留，不能删除或用默认值
代替。

当前固件中的 J3、J4、J6 仍有非零机械 offset。要采用本文的 OpenArmX 全零参考姿态，
必须将这三个通道也改为 `0.0f`。

### 7.3 编译并烧录 USB M5 固件

```bash
cd /home/alientek/openflex/ker/firmware/M5
pio run -e usb
pio run -e usb --target upload
```

这里改烧的是 M5 主控固件，不是编码器 ID 1-7 固件。

### 7.4 确定 KER 参考姿态

1. 启动 OpenArmX fake hardware。
2. 给右臂发送全零 controller 命令。
3. 观察 RViz 中右臂的 J1-J7 全零姿态。
4. 让 KER 右臂模仿该姿态。
5. 特别核对 J3、J5、J7 的旋转方向，不能只看末端位置。

### 7.5 只对 1-7 执行 Zero

1. M5 保持 STANDBY。
2. 在屏幕上依次选择柱状通道 1-7。
3. 确认 8-16 没有被选择。
4. 点击 `Confirm Zero`。
5. 确认写入。
6. 不点击 `Zero Reset (All)`。

缺失通道的 `raw_deg` 无效，Zero All 会污染 8-16 的 NVS 标定数据。

### 7.6 用官方 CLI 验证固件输出

先停止 ROS KER 驱动，确保 USB 没有被占用：

```bash
cd /home/alientek/openflex/ker/openarm_ker-main
source .venv/bin/activate
openarm-ker-cli stream
```

参考姿态下要求：

- `angles[0]` 至 `angles[6]` 接近 `0 deg`。
- `errors[0]` 至 `errors[6]` 为 `False`。
- `errors[7]` 至 `errors[15]` 为 `True`。
- 缓慢移动一个关节时，主要只有对应通道变化。

退出 CLI 后才能启动 ROS 驱动。

## 8. 启动 KER 驱动

保持 OpenArmX fake hardware launch 运行。新开终端：

```bash
cd /home/alientek/openflex/openarmx_ker_sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/alientek/openflex/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=usb \
  -p drop_command_on_sensor_error:=false
```

只在 fake hardware 仿真中使用最后一行参数。

检查：

```bash
ros2 service call /ker/ping_usb std_srvs/srv/Trigger
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/joint_states --once
ros2 topic echo /ker/error_mask --once
ros2 topic echo /ker/right_arm/joint_command --once
```

预期：

- Ping 成功。
- `/ker/error_mask = 65408`。
- 右臂目标消息包含 J1-J7 和固定右夹爪，共 8 项。
- 此时 bridge 尚未启动，OpenArmX RViz 不应跟随 KER。

## 9. 启动右臂仿真 bridge

启动前让 KER 回到参考姿态，确认右臂目标 J1-J7 都接近 0 rad。

新开终端：

```bash
cd /home/alientek/openflex/openarmx_ker_sim_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 run openflex_teleop_ker ker_arm_bridge_node --ros-args \
  --params-file /home/alientek/openflex/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p enable_safety_check:=true \
  -p max_joint_diff_rad:=0.873 \
  -p interpolation_duration:=3.0 \
  -p target_timeout_s:=0.25 \
  -p left_controller_topic:=/ker/sim/disabled_left_controller/commands
```

预期日志：

```text
[right] safe takeover started
```

如果出现：

```text
[right] takeover rejected: jointN differs by ...
```

说明 KER 与 fake hardware 初始姿态差超过 50 度。停止 bridge，让 KER 和 RViz 回到
参考姿态后重新启动。不要关闭安全检查绕过。

## 10. RViz 逐轴验证

bridge 成功接管后，每次只移动一个关节，先从约 5 度开始。

| KER 通道 | ROS 源名称 | OpenArmX 目标 | 验收 |
|---|---|---|---|
| 1 | `ker_right_joint1` | `openarmx_right_joint1` | |
| 2 | `ker_right_joint2` | `openarmx_right_joint2` | |
| 3 | `ker_right_joint3` | `openarmx_right_joint3` | |
| 4 | `ker_right_joint4` | `openarmx_right_joint4` | |
| 5 | `ker_right_joint5` | `openarmx_right_joint5` | |
| 6 | `ker_right_joint6` | `openarmx_right_joint6` | |
| 7 | `ker_right_joint7` | `openarmx_right_joint7` | |

每轴检查：

1. KER 正向移动，RViz 对应关节正向运动。
2. KER 回到参考姿态，RViz 回到接近 0 rad。
3. 其他关节没有明显串动。
4. `/joint_states` 和 `/ker/right_arm/joint_command` 数值一致到允许误差。
5. 快速停止 KER 后，RViz 不持续漂移。

可同时观察：

```bash
ros2 topic echo /ker/right_arm/joint_command
ros2 topic echo /right_forward_position_controller/commands
ros2 topic echo /joint_states
```

## 11. 方向或零位不正确时如何修正

### 11.1 固定零位误差

如果参考姿态 Zero 后某个通道仍不是预期角度，检查：

- `mech_joint_offset` 是否已经烧录为 0。
- Zero 时是否选中了正确通道。
- KER 是否确实处于参考姿态。
- M5 NVS 是否在重启后保存了新 Zero。

本方案 Zero 瞬间输出应等于 `mech_joint_offset`，所以设为 0 后应接近 0 度。

### 11.2 方向相反

方向问题在 M5 固件 `ENCODER_CONFIG[].invert` 中修正。修改后重新烧录，并重新对该通道
执行 Confirm Zero。不要同时修改固件 `invert` 和 ROS `joint_scales`，否则容易重复反向。

### 11.3 小角度比例问题

磁编码器角度比例理论上固定为 1:1。若 KER 移动 10 度而 RViz 明显不是约
`0.1745 rad`，先检查编码器机械传动、安装和通道 ID，不使用 ROS scale 掩盖机械问题。

## 12. 停止顺序

按以下顺序停止：

1. `Ctrl+C` 停止 `ker_arm_bridge_node`。
2. `Ctrl+C` 停止 `ker_driver_node`，驱动会发送 STANDBY。
3. 停止 OpenArmX fake hardware launch。
4. 最后断开 M5。

停止后确认：

```bash
ros2 node list
```

不能残留旧 bridge 后再启动第二套测试。

## 13. 常见问题

### `/ker/joint_states` 有数据，但 `/ker/right_arm/joint_command` 没有数据

检查驱动是否仅在 fake 仿真命令行覆盖：

```text
drop_command_on_sensor_error:=false
```

因为当前 `error_mask=65408`，默认保护会禁止目标发布。

### 右臂目标有数据，但 RViz 不动

检查：

```bash
ros2 control list_controllers
ros2 topic info /right_forward_position_controller/commands --verbose
ros2 topic echo /right_forward_position_controller/commands --once
```

确认使用的是 `forward_position_controller`，而不是 trajectory controller。

### bridge 提示等待 `/joint_states`

`joint_state_broadcaster` 尚未 active，或 fake hardware launch 未正常启动。

### 左臂在动

立即停止 bridge，确认启动参数中存在：

```text
left_controller_topic:=/ker/sim/disabled_left_controller/commands
```

并确认没有其他节点直接向左臂 controller 发布。

### 右夹爪打开但不跟随

这是预期行为。通道 8 缺失，夹爪只使用固定占位值满足 8-DOF controller。

### M5 显示 8-16 ERR

这是当前硬件缺失导致的预期结果。只要 1-7 不显示 ERR，就不影响本文的右臂 fake
仿真测试。

## 14. 实机隔离警告

本文使用了以下仅仿真参数组合：

```text
use_fake_hardware=true
drop_command_on_sensor_error=false
left_controller_topic=/ker/sim/disabled_left_controller/commands
```

切换到 OpenArmX 实机前必须撤销 `drop_command_on_sensor_error=false`。只有 1-7 通道时
不允许使用当前配置控制实机，因为：

- 通道 8 夹爪无反馈。
- 通道 9-16 整条左臂无反馈。
- 全局错误保护被临时关闭。
- 当前 bridge 还没有正式的 `right_arm_only` 产品配置。

本文通过不等于实机遥操通过。

## 15. 验收清单

- [ ] OpenArmX 实机电机断电或 CAN 断开。
- [ ] OpenArmX launch 明确使用 `use_fake_hardware:=true`。
- [ ] 右臂 forward position controller 为 active。
- [ ] 手动 8 项命令能驱动 RViz 右臂。
- [ ] M5 通道 1-7 的 `mech_joint_offset` 已按零姿态设为 0。
- [ ] 只对通道 1-7 执行 Confirm Zero。
- [ ] 没有执行 Zero All。
- [ ] 官方 CLI 中 `angles[0:7]` 在参考姿态接近 0 度。
- [ ] 官方 CLI 中 `errors[0:7]` 全为 False。
- [ ] ROS `/ker/error_mask` 为 `65408`。
- [ ] 驱动的错误保护只在 fake 仿真命令行临时关闭。
- [ ] 左臂 controller 输出已重映射到废弃话题。
- [ ] bridge 初始安全检查保持开启。
- [ ] J1-J7 已逐轴验证通道、零点、方向和弧度单位。
- [ ] 右夹爪未被误记为已标定。
- [ ] 没有把本次结果用于 OpenArmX 实机放行。

完成以上项目，即可确认 KER 通道 1-7 能通过现有 ROS 2 节点驱动 OpenArmX fake
hardware，并在 RViz 中正确显示右臂 J1-J7 的跟随运动。

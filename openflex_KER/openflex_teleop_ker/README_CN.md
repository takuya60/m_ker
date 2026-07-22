# openflex_teleop_ker

该包将 OpenArm KER 2.0 的 USB、串口或 WiFi 数据适配到 OpenFlex ROS 2 控制链路。使用官方夹具时可保留官方参考角；没有夹具、使用 OpenFlex 双臂下垂姿态标定时需要按标定手册修改固件参考角。

## 数据链路

```text
KER 固件 (16 路角度)
  -> /ker/joint_states                    # KER 原始物理角度，rad
  -> /ker/{left,right}_arm/joint_command  # OpenFlex 命名的目标角度
  -> 安全检查和渐进接管
  -> /{left,right}_forward_position_controller/commands
```

通道顺序与官方 KER 2.0 一致：右臂 7 轴、右夹爪、左臂 7 轴、左夹爪。固件已经处理编码器方向、零偏、机械偏移和跳变；上位机只进行度到弧度转换。夹爪按照 OpenFlex 驱动定义映射为 `0.0` 闭合、`0.044` 张开，可在 YAML 中调整。

## 编码器标定

没有官方标定夹具、需要使用双臂下垂姿态标定时，请先阅读完整的
[OpenFlex KER 编码器标定手册](CALIBRATION_CN.md)。

代码中的 OpenFlex `home`、仿真初始姿态和 `return_to_zero()` 都将左右臂 J1-J7
定义为 `0 rad`。当前单姿态标定会先确认实机下垂时 `/joint_states` 接近全零，再把
`ker/firmware/M5/include/Common.h` 中 14 个手臂通道的
`ENCODER_CONFIG[].mech_joint_offset` 改为 `0.0f`，重新烧录后让 KER 在相同下垂
姿态执行 Zero All。完整操作和夹爪坐标说明以标定手册为准。

## 编译

将 `openflex_teleop_ker` 和 `openflex_ker_joint_panel` 放入 ROS 2 工作空间的 `src` 后执行：

```bash
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select openflex_teleop_ker openflex_ker_joint_panel
source install/setup.zsh
```

USB 模式需要 KER 官方 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 启动

先启动 OpenFlex，并确认左右臂使用 `forward_position_controller`，再让 KER 与机器人处于相近姿态：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py
```

串口模式：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py transport:=serial
```

WiFi 模式要求 M5 和机器人主机位于同一个可信局域网。先按固件 README 创建
`include/WiFiConfig.h` 并烧录 `wifi` 环境，然后启动：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py transport:=wifi
```

默认连接 `openarm-ker.local:19090`。若机器人网络不支持 mDNS，在
`config/openflex_ker.yaml` 的 `wifi_host` 中填写 M5 屏幕显示的 IPv4 地址。固件烧录的
传输模式必须与 launch 的 `transport` 一致；固件不会在 USB 和 WiFi 之间自动切换。

首次硬件测试不要关闭安全检查。若任一关节初始差值超过 `0.873 rad`，桥接节点拒绝接管；调整双方姿态后重启节点。调试仿真时才可使用：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py enable_safety_check:=false
```

## 检查

```bash
ros2 topic echo /ker/metadata --once
ros2 service call /ker/ping std_srvs/srv/Trigger
ros2 service call /ker/ping_usb std_srvs/srv/Trigger
ros2 service call /ker/ping_wifi std_srvs/srv/Trigger
ros2 topic echo /ker/joint_states --once
ros2 topic echo /ker/error_mask
ros2 topic echo /ker/left_arm/joint_command --once
ros2 topic echo /left_forward_position_controller/commands --once
```

`/ker/error_mask` 为 16 位传感器故障位图。默认任意位非零时停止更新双臂目标，控制器桥同时具有 0.25 秒命令超时保护。

`/ker/ping` 检查当前配置的传输，`/ker/ping_usb` 和 `/ker/ping_wifi` 强制检查指定传输。
返回消息包含端点、完整连接与 schema 握手耗时、硬件版本、固件版本和更新时间。

# m_ker

`m_ker` 是 KER 主从遥操作项目，包含编码器固件、M5 主控固件、ROS 2 遥操节点、
RViz Panel 和烧录工具。支持 USB、Serial 和 WiFi，将 KER 的 16 路角度映射到
OpenArmX 双臂关节与夹爪。

## 数据通路

```text
编码器 -> RS485 -> M5 -> USB/WiFi -> ker_driver_node
       -> /ker/*/joint_command -> ker_arm_bridge_node
       -> ros2_control -> OpenArmX / RViz
```

## 目录结构

```text
m_ker/
├── firmware/
│   ├── encoder/                         # ATtiny1616 编码器固件
│   └── M5/                              # M5Stack CoreS3 主控固件
├── openflex_KER/
│   ├── openflex_teleop_ker/             # ROS 2 driver、bridge、launch、配置
│   └── openflex_tool/
│       └── openflex_ker_joint_panel/    # RViz 关节与参数 Panel
├── tool/                                # Windows/Linux 固件烧录与编码器测试工具
└── docs/                                # 固件和 ROS 2 中文手册
```

## 快速部署

### 1. 环境

- Ubuntu 22.04
- ROS 2 Humble
- 已安装 OpenArmX 的 `openarmx_description` 和 `openarmx_bringup`

将仓库放入 ROS 2 工作空间的 `src`：

```bash
cd /home/openflex/openflex_all/openflex_ws/src
git clone https://github.com/takuya60/m_ker.git
```

### 2. 编译 ROS 2 包

```bash
cd /home/openflex/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  openflex_teleop_ker openflex_ker_joint_panel
source install/setup.bash
```

每个新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/openflex_all/openflex_ws/install/setup.bash
```

### 3. 烧录固件

图形工具：

```bash
cd /home/openflex/openflex_all/openflex_ws/src/m_ker
python3 tool/encoder_tool.py
```

工具包含编码器和 M5 的源码烧录、HEX/BIN 直烧及编码器 ID/角度测试。详细步骤见：

- [M5 固件手册](docs/firmware/M5_FIRMWARE_CN.md)
- [编码器固件手册](docs/firmware/ENCODER_FIRMWARE_CN.md)

## 快速启动

### RViz 一键仿真

启动 fake hardware、KER driver、bridge 和 RViz：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

临时覆盖连接参数：

```bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py \
  transport:=wifi \
  wifi_host:=192.168.3.114 \
  wifi_port:=19090
```

### 单独启动双臂 RViz

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller \
  enable_forward_effort:=false
```

另一个终端启动 KER 右臂遥操：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py
```

### 真机双臂遥操

终端 1 启动真机、controller 和 RViz：

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  enable_forward_effort:=false
```

终端 2 启动 KER：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py \
  transport:=wifi \
  wifi_host:=openarm-ker.local \
  wifi_port:=19090 \
  drop_command_on_sensor_error:=false \
  max_joint_velocity_rad_s:=3.0 \
  max_gripper_velocity_m_s:=0.3
```

M5 需要处于 `STREAM` 状态。启动前确认 CAN 接口、控制器、急停和机械臂周围环境。

## RViz Panel

在 RViz 中选择：

```text
Panels -> Add New Panel -> openflex_ker_joint_panel/JointAnglePanel
```

Panel 可查看关节角度、数据频率和连接状态，也可在线修改滤波、关节速度、夹爪速度与
目标超时参数。

## 常用检查

```bash
ros2 topic echo /ker/error_mask --once
ros2 topic echo /ker/joint_states --once
ros2 topic hz /ker/joint_states
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

查看 launch 支持的覆盖参数：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py --show-args
```

## 详细文档

- [ROS 2 遥操手册](docs/ros2/ROS2_TELEOP_CN.md)
- [全部文档索引](docs/README_CN.md)
- [ROS 2 包详细说明](openflex_KER/openflex_teleop_ker/README_CN.md)

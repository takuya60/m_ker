# KER 仅安装 1-7 号关节时的分阶段测试与遥操计划

日期：2026-07-22

## 1. 目的与当前条件

当前只有编码器节点 ID 1-7，并且已经分别烧录对应 ID 的编码器固件。本计划先验证
RS-485、M5、USB 和上位机之间的信号通路，再验证 OpenFlex ROS 2 节点，然后改烧
WiFi 版 M5 固件并验证网络传输。所有测试程序均运行在机器人主机上。

当前固件通道定义如下：

| KER 通道 | OpenFlex 含义 | 当前状态 |
|---|---|---|
| 1-7 | 右臂 J1-J7 | 已安装，必须正常 |
| 8 | 右夹爪 | 未安装，报错属于预期 |
| 9-15 | 左臂 J1-J7 | 未安装，报错属于预期 |
| 16 | 左夹爪 | 未安装，报错属于预期 |

M5 固件仍然发布固定的 16 路 `angles` 和 16 路 `errors`，不会因为只安装 7 路而把
schema 改成 7 路。理想情况下 1-7 路 `errors=false`，8-16 路 `errors=true`。

ROS 2 `/ker/error_mask` 的 bit0 对应通道 1，bit15 对应通道 16。因此只有 1-7 正常时，
预期值为：

```text
二进制：1111 1111 1000 0000
十六进制：0xFF80
十进制：65408
```

该错误值在当前不完整硬件状态下是预期结果，不代表 USB 或 WiFi 传输失败。

## 2. 重要安全限制

1. 阶段一至阶段四只测试 KER，不启动 OpenFlex 控制桥，不让机器人跟随。
2. 不使用完整 launch 文件做前期测试，因为它会同时启动
   `openflex_ker_arm_bridge`。只单独运行 `ker_driver_node`。
3. 8-16 通道缺失时不要点击 M5 的 `Zero Reset (All)`。无效通道没有可靠的
   `raw_deg`，Zero All 会把无效数据写入这些通道的 NVS 标定值。
4. 本轮信号测试原则上不需要重新 Zero。确实需要标定时，只选择屏幕上的通道 1-7，
   再执行 `Confirm Zero`，并按照现有标定文档规定的右臂参考姿态操作。
5. 不要为了产生遥操目标而把 `drop_command_on_sensor_error` 直接改成 `false`。
   这样会同时放行缺失的右夹爪和整条左臂的无效角度。
6. 在完成本文第五阶段的软件隔离改造前，不进行实机遥操。

## 3. 测试记录表

测试过程中保存下表。每次只缓慢转动一个 KER 关节，其他关节保持不动。

| 通道 | 对应关节 | 官方 CLI 有数据 | ROS 有数据 | WiFi 有数据 | 运动方向正确 | 静止抖动/异常 |
|---|---|---|---|---|---|---|
| 1 | 右 J1 | | | | | |
| 2 | 右 J2 | | | | | |
| 3 | 右 J3 | | | | | |
| 4 | 右 J4 | | | | | |
| 5 | 右 J5 | | | | | |
| 6 | 右 J6 | | | | | |
| 7 | 右 J7 | | | | | |

另外记录：

- M5 固件版本和硬件版本。
- USB 连续测试频率、WiFi 连续测试频率。
- USB 和 WiFi 下每个关节静止值及同一姿态的差值。
- 是否出现跳变停止、周期性断流、错误位变化或 M5 重启。

## 4. 阶段零：接线与 M5 静态检查

### 4.1 操作

1. 断电检查 RS-485 总线，确认节点 ID 1-7 唯一，没有两个节点烧录相同 ID。
2. 检查电源、GND、RS-485 A/B 极性和总线端接。
3. 给 KER 上电，不连接机器人控制器。
4. 观察 M5 的 16 个通道条形图。
5. 依次缓慢转动 1-7 号关节，每次只动一个。

### 4.2 预期结果

- 1-7 号条形图分别响应，不能出现转动 J3 而 J4 变化的 ID 串位现象。
- 8-16 显示错误属于预期。
- 1-7 不应持续显示 `ERR`。
- 关节静止时数据应基本稳定；缓慢转动时应连续变化，不能频繁跳变 360 度。

### 4.3 通过条件

1-7 通道全部能独立、连续响应，M5 不重启，且没有通道串号。任意一项不满足时先检查
编码器 ID、供电和 RS-485 总线，不进入 USB 上位机测试。

## 5. 阶段一：使用官方上位机验证 USB 通路

本阶段使用官方 `openarm_ker` CLI，不使用 Dora 遥操节点。CLI 只读取设备 metadata、
schema 和原始 stream，更适合作为最小化信号测试工具。

### 5.1 确认 M5 固件模式

M5 必须烧录 `usb` 环境。编码器节点 1-7 的固件不需要重烧；这里选择的是 M5 到主机
之间的传输固件。

连接 USB 后检查：

```bash
lsusb | grep -i 303a
```

应看到 `303a:4002`。若普通用户无权限，按官方规则配置一次：

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 M5 后再测试。

### 5.2 安装本地官方上位机

官方包要求 Python 3.11 或更高。优先在机器人主机的独立虚拟环境中安装仓库内版本：

```bash
cd /home/alientek/openflex/ker/openarm_ker-main
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e .
```

若系统 `python3` 低于 3.11，应使用机器人主机已有的 Python 3.11 或 `uv` 环境，不能
升级或替换 ROS 2 所使用的系统 Python。

### 5.3 Ping 测试

```bash
openarm-ker-cli ping
```

记录 `fw`、`hw`、`updated`。Ping 成功只说明 USB 双向命令、PING 响应和 schema 解析
正常，还不能证明 1-7 的连续角度均正常。

### 5.4 Stream 测试

```bash
openarm-ker-cli stream
```

依次缓慢转动通道 1-7，观察字典中的：

- `angles[0]` 到 `angles[6]` 分别对应通道 1-7。
- `errors[0]` 到 `errors[6]` 应为 `false`。
- `errors[7]` 到 `errors[15]` 应为 `true`，这是缺失通道的预期结果。
- `timestamp` 应持续更新。

按 `Ctrl+C` 正常退出，确保 CLI 向 M5 发送 STANDBY 并释放 USB。

### 5.5 通过条件

- Ping 连续执行 3 次均成功。
- Stream 连续运行至少 5 分钟不退出、不冻结。
- 1-7 逐个运动时只有对应角度发生主要变化。
- 1-7 的错误位保持 false；8-16 的错误位保持 true。

如果 Ping 成功但 1-7 某一路错误，问题更可能在该编码器节点或 RS-485，而不是 USB。

## 6. 阶段二：使用 ROS 2 节点验证 USB 通路

### 6.1 构建

在机器人主机的 ROS 2 工作空间中放置或链接 `openflex_teleop_ker` 和 RViz 面板包，执行：

```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select openflex_teleop_ker openflex_ker_joint_panel
source install/setup.bash
```

先退出官方 CLI，确保 USB 没有被另一个进程占用。

### 6.2 只启动驱动节点

不要运行 `openflex_ker_teleop.launch.py`。单独启动：

```bash
ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/alientek/openflex/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=usb
```

在其他终端分别检查：

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 service call /ker/ping_usb std_srvs/srv/Trigger
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/joint_states --once
ros2 topic echo /ker/error_mask
```

### 6.3 预期结果

- USB Ping 返回 `success=true`，endpoint 为 `USB 0x303a:0x4002`。
- `/ker/joint_states` 有 16 个名称和 16 个位置。
- 列表前 7 项是 `ker_right_joint1` 至 `ker_right_joint7`，单位为 rad。
- `/ker/error_mask` 稳定在 `65408`，即 `0xFF80`。
- 因 `drop_command_on_sensor_error=true`，当前不应产生左右臂目标消息。这是安全保护正常，
  不是 ROS 驱动故障。

可打开 RViz2，添加 `openflex_ker_joint_panel/JointAnglePanel`，使用 `USB Ping` 并观察
右臂 J1-J7。此时不要启动 OpenFlex controller bridge。

### 6.4 通过条件

- ROS USB Ping 连续 3 次成功。
- `/ker/joint_states` 连续运行至少 10 分钟，频率稳定，无周期性断连。
- 1-7 的 ROS 角度变化与官方 CLI 记录一致。
- 错误位严格符合当前硬件：1-7 无错误，8-16 有错误。

若 `/ker/error_mask` 不是 `65408`，先转换为十六进制并定位新增错误位。例如 bit2 为 1
表示通道 3 异常，不能把整个非零值简单视为“只有缺失通道”。

## 7. 阶段三：改烧 WiFi M5 固件

只有 USB 官方 CLI 和 ROS USB 均通过后才进入本阶段。

### 7.1 配置 WiFi

```bash
cd /home/alientek/openflex/ker/firmware/M5
cp include/WiFiConfig.example.h include/WiFiConfig.h
```

编辑 `include/WiFiConfig.h`：

```cpp
#define KER_WIFI_SSID     "机器人局域网名称"
#define KER_WIFI_PASSWORD "机器人局域网密码"
#define KER_WIFI_HOSTNAME "openarm-ker"
#define KER_WIFI_PORT     19090
```

主机和 M5 必须在同一局域网，网络不能启用无线客户端隔离。该 TCP 服务没有认证或
加密，只能使用可信局域网，不能把端口映射到公网。

### 7.2 编译和烧录

```bash
pio run -e wifi
pio run -e wifi --target upload
```

这里仅改烧 M5 主控固件，不需要重烧 1-7 号编码器节点固件。重启后记录 M5 屏幕上的
IPv4 地址、`WIFI` 状态和客户端状态。

### 7.3 网络基础检查

```bash
getent hosts openarm-ker.local
ping -c 3 openarm-ker.local
nc -vz openarm-ker.local 19090
```

如果 mDNS 解析失败但 M5 已取得 IP，直接在 YAML 的 `wifi_host` 中填写该 IPv4 地址。
`nc` 只做端口探测，测试完应退出；WiFi 固件同一时刻只允许一个 TCP 客户端。

### 7.4 通过条件

- M5 能稳定获取 IP，不反复掉线或重启。
- 机器人主机可以解析 mDNS 或直接访问固定 IP。
- TCP `19090` 可连接。

## 8. 阶段四：使用 ROS 2 节点验证 WiFi 通路

仍然只运行驱动节点，不启动控制桥。先结束 `nc`、USB CLI 和其他 ROS KER 驱动，确保
只有一个 WiFi client。

```bash
ros2 run openflex_teleop_ker ker_driver_node --ros-args \
  --params-file /home/alientek/openflex/openflex_KER/openflex_teleop_ker/config/openflex_ker.yaml \
  -p transport:=wifi \
  -p wifi_host:=openarm-ker.local \
  -p wifi_port:=19090
```

检查：

```bash
ros2 service call /ker/ping_wifi std_srvs/srv/Trigger
ros2 topic hz /ker/joint_states
ros2 topic echo /ker/joint_states --once
ros2 topic echo /ker/error_mask
```

在 RViz 面板点击 `WiFi Ping`，确认返回 endpoint、latency、HW、FW 和 updated。依次转动
1-7，填写测试记录表，并将同一姿态下的 WiFi 角度和 USB 角度比较。

### 8.1 通过条件

- WiFi Ping 连续 10 次成功，无随机超时。
- 连续 stream 至少 30 分钟，不断流、不重连、不触发 M5 重启。
- `/ker/error_mask` 与 USB 相同，稳定为 `65408`。
- 1-7 的角度、方向和 USB 结果一致；传输方式不能改变标定结果。
- 关节静止和缓慢运动时，话题频率、延迟满足实际遥操要求。

如 WiFi 失败而 USB 正常，优先检查 mDNS、AP 客户端隔离、信号强度、DHCP 地址变化和
TCP 单客户端占用，不重新标定编码器。

## 9. 阶段五：进入遥操前必须完成的改造

### 9.1 当前代码为什么不能直接遥操

当前 `ker_driver_node.py` 的安全策略是：16 路中任意一路错误，左右臂目标都不发布。
因此只有 1-7 时，`0xFF80` 会正确阻止遥操。

把 `drop_command_on_sensor_error` 改为 `false` 也不可接受，因为当前 pose processor 会
继续根据缺失的通道 8-16 生成右夹爪和整条左臂目标，桥接节点还会向左右 8-DOF
controller 发布命令。

### 9.2 推荐选择

优先选择以下方案之一：

**方案 A：补齐 8-16 通道后进行双臂遥操。**

这是风险最低的路径。补齐后要求 `/ker/error_mask=0`，再按完整标定文档完成 Zero、
姿态核对和安全接管。

**方案 B：实现“仅右臂 J1-J7”诊断遥操模式。**

如果必须在现有硬件上测试右臂，进入实机前至少要完成以下代码改造和测试：

1. 驱动增加 `enabled_sides` 或 `active_channel_mask`，只把通道 1-7 作为右臂有效输入。
2. 错误保护按手臂拆分，右臂只检查 bit0-bit6；不能全局忽略 bit7-bit15。
3. 完全禁止左臂目标发布和左臂 controller 输出。
4. 右夹爪通道 8 缺失时，不使用 KER 的无效值；桥接时保持机器人当前夹爪位置。
5. 右臂安全接管仍保留 `max_joint_diff_rad=0.873`、3 秒插值和 0.25 秒目标超时。
6. launch 增加明确的 `right_arm_only` 配置，默认仍保持完整双臂安全模式。
7. 为通道 mask、单臂错误、夹爪保持和左臂零输出增加自动测试。

由于 OpenFlex 当前右侧 `forward_position_controller` 是 8-DOF，方案 B 不能简单少发一个
数值；必须用机器人 `/joint_states` 中当前的右夹爪值补齐第 8 项并持续保持。

### 9.3 遥操实施顺序

完成方案 A 或 B 后，再按以下顺序实施：

1. 机器人断使能或使用仿真，先验证 controller 输出的关节顺序和数量。
2. KER 与 OpenFlex 右臂摆到接近姿态，逐项比较 7 个关节差值。
3. 保持安全检查开启，首次测试将速度和运动范围限制在较小范围。
4. 设置硬件急停观察员，操作者先只移动 J1，再逐轴增加到 J7。
5. 验证网络断开、M5 断电和 ROS 节点退出时，0.25 秒超时能停止新命令。
6. USB 遥操通过后，再使用相同标定切换到 WiFi 遥操；不要因传输方式变化重新 Zero。

## 10. 最终放行清单

- [ ] 编码器节点 ID 1-7 唯一且与物理关节一致。
- [ ] 未对缺失通道执行 Zero All。
- [ ] 官方 USB Ping 3 次通过。
- [ ] 官方 USB Stream 5 分钟通过。
- [ ] ROS USB Stream 10 分钟通过。
- [ ] `/ker/error_mask` 稳定为 `0xFF80`，没有 1-7 的错误位。
- [ ] WiFi Ping 10 次通过。
- [ ] WiFi Stream 30 分钟通过。
- [ ] USB/WiFi 同姿态角度一致。
- [ ] 已选择补齐硬件或完成右臂单臂软件隔离改造。
- [ ] 未通过关闭全局错误保护绕过缺失通道。
- [ ] 控制输出已先在断使能或仿真环境验证。
- [ ] 实机首次接管保留角差限制、插值、命令超时和急停措施。

只有以上项目全部满足，才进入对应硬件范围内的遥操测试。

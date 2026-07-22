# 官方 Dora KER 与 OpenArm RViz 仿真验证指导

日期：2026-07-22

> 状态：该方案已停止推进，仅保留为接口分析记录。当前采用
> `KER_OPENARMX_RVIZ_SIM_CALIBRATION_GUIDE_CN.md` 中的 OpenArmX fake hardware
> RViz 流程。

## 1. 目标与边界

本阶段只做以下工作：

1. 使用官方 `openarm_ker` 验证 M5 USB 数据。
2. 使用官方 `dora-openarm-ker` 节点读取 KER。
3. 使用官方 `openarm_description` 在 RViz2 中显示 OpenArm v2.0 右臂模型。
4. 当前只使用 KER 通道 1-7，对应右臂 J1-J7。
5. 角度参考和方向修正只放在 M5 固件标定层完成。

本阶段明确不做：

- 不使用 `openflex_teleop_ker`。
- 不使用我们编写的 ROS 2 驱动或 RViz panel。
- 不连接或控制 OpenArmX/OpenFlex 实机。
- 不启动 `dora-openarm` follower 节点，因为它会连接真实 OpenArm CAN 驱动。
- 不修改官方 `dora-openarm-ker` 的角度映射。
- 不对缺失的 8-16 通道执行 Zero All。

## 2. 必须先理解的接口边界

### 2.1 官方 Dora KER 的输出

官方 `dora-openarm-ker` 读取 M5 的 16 路角度后输出：

```text
follower_position_right = {qpos: float32[8]}
follower_position_left  = {qpos: float32[8]}
```

右侧前 7 项是通道 1-7 转换成的弧度，第 8 项是通道 8 夹爪。左侧使用通道 9-16。
该节点不会根据 `errors` 阻止输出，因此即使只有 1-7，它仍会生成右夹爪和左臂数据，
只是这些值来自缺失通道的保持值，不能用于控制或验收。

### 2.2 RViz 的输入

官方 `openarm_description` 启动：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz2

模型姿态由 ROS 2 `sensor_msgs/msg/JointState` 驱动。右臂无夹爪 preset 使用以下名称：

```text
openarm_right_joint1
openarm_right_joint2
...
openarm_right_joint7
```

### 2.3 为什么官方 Dora KER 不能直接连接 RViz

Dora KER 输出只有 `qpos` 数组；ROS `JointState` 至少需要 `name`、`position`、
`velocity`、`effort` 和 `header` 的消息结构。Dora dataflow YAML 可以连接 topic，但不会
自动完成以下转换：

- 把 `qpos` 字段改名为 `position`。
- 添加 7 个 OpenArm 关节名称。
- 从 float32 转成 ROS JointState 使用的 float64 序列。
- 丢弃当前无效的第 8 项夹爪。
- 生成 ROS Header。

因此，在“主机侧不新增任何适配代码”的严格条件下，可以分别完成：

1. 官方 Dora KER 数据链路验证。
2. 官方 OpenArm RViz 模型与关节名称验证。

但不能把这两项称为“KER 已经实时驱动 RViz”。真正的端到端 KER -> RViz 至少需要
一个 Dora Arrow 到 ROS JointState 的格式适配器。它可以不是我们自己的 ROS 节点，
但仍属于新增主机侧软件，和“只改 M5 标定”的当前约束冲突。

## 3. 本阶段推荐架构

```text
链路 A：信号与官方算法验证

KER 编码器 1-7
  -> M5 USB 固件
  -> 官方 openarm_ker
  -> 官方 dora-openarm-ker
  -> Dora follower_position_right

链路 B：官方 OpenArm 模型验证

官方 openarm_description
  -> joint_state_publisher_gui
  -> /joint_states
  -> robot_state_publisher
  -> RViz2 OpenArm v2.0 right_arm
```

先让两条链路分别通过。需要实时联动时，再单独审批适配层，不应通过修改官方 Dora
角度或混用 OpenArmX 模型来绕过接口问题。

## 4. 使用的官方仓库版本

编写本文时核对的官方仓库和提交为：

| 仓库 | 参考提交 | 用途 |
|---|---|---|
| `enactic/openarm` | `535b54f6ff79` | 官方仓库索引与文档 |
| `enactic/openarm_ker` | 当前目录内官方副本 | M5 主机端协议 |
| `enactic/dora-openarm-ker` | 当前目录内官方副本 | KER Dora leader |
| `enactic/openarm_description` | `6c7b720f1ba4` | OpenArm URDF 与 RViz launch |
| `enactic/dora-openarm-data-collection` | `e101394c4938` | 官方 KER dataflow 参考 |

官方 data collection 的真实 follower 是 `dora-openarm`，会操作真实机械臂，不适合本阶段。

## 5. M5 固件标定

### 5.1 标定目标

官方 Dora 对右臂 J1-J7 不添加主机端 offset：

```text
qpos_right[i] = radians(M5_angle_deg[i]), i = 0..6
```

所以固件输出必须已经是希望 OpenArm 模型使用的物理关节角。建议本阶段把 OpenArm
RViz 的全零姿态作为参考姿态：KER 模仿该右臂姿态时，通道 1-7 应输出接近 `0 deg`。

### 5.2 修改位置

文件：

```text
/home/alientek/openflex/ker/firmware/M5/include/Common.h
```

只修改 `ENCODER_CONFIG` 的通道 1-7。主要字段为第 4 项
`mech_joint_offset`：

```cpp
// invert, mech_min, mech_max, mech_joint_offset, skip_jump_detect
{false, -80.0f, 160.0f, 0.0f, false}, // ch1 right J1
```

若选定的参考姿态在 OpenArm RViz 中是 J1-J7 全零，则通道 1-7 的
`mech_joint_offset` 应设为 `0.0f`。不要修改通道 8-16，也不要为了让画面看起来正确而
在 Dora 或 ROS 中增加第二套 offset。

当前文件中 J3、J4、J6 仍存在非零机械 offset。使用“参考姿态等于全零”的方案时，
这些通道也必须按标定文档改为 `0.0f` 后重新编译 M5 固件。

### 5.3 Zero 操作

1. 在官方 RViz 中先用 GUI 把右臂 J1-J7 全部设为 0。
2. 观察 OpenArm 模型的全零姿态。
3. 让 KER 右臂尽量模仿该姿态。
4. 在 M5 屏幕中只选中柱状通道 1-7。
5. 点击 `Confirm Zero` 并确认。
6. 不点击 `Zero Reset (All)`。

缺失通道没有可靠的 `raw_deg`。Zero All 会把无效数据写入 8-16 的 NVS 标定区域，
因此当前硬件状态必须使用通道选择 Zero。

### 5.4 标定后检查

```bash
cd /home/alientek/openflex/ker/openarm_ker-main
source .venv/bin/activate
openarm-ker-cli stream
```

参考姿态下要求：

- `angles[0:7]` 接近 0 度。
- `errors[0:7]` 全部为 `False`。
- `errors[7:16]` 为 `True`，这是缺失通道的预期结果。
- 依次缓慢移动 J1-J7，只有对应主要通道变化。
- 正方向与 OpenArm RViz GUI 中该关节增加时的运动方向一致。

若方向相反，先核对编码器机械安装和 `invert`。不要通过把正负号写到 Dora 节点中解决。

## 6. 安装官方 Dora KER 环境

官方 Dora KER 使用 Python 3.11 以上；本机已经准备 Python 3.12。为 Dora 单独创建环境，
不要使用 ROS 2 的系统 Python 3.10 环境：

```bash
cd /home/alientek/openflex/ker/dora-openarm-ker-main
uv venv --python 3.12 .venv
source .venv/bin/activate
```

优先使用当前目录中的官方 `openarm_ker` 副本，避免 Dora 安装时又下载一个不同版本：

```bash
uv pip install -e ../openarm_ker-main
uv pip install "dora-rs==0.5.0" "dora-rs-cli==0.5.0" pyarrow numpy
uv pip install -e . --no-deps
```

验证：

```bash
python --version
dora --version
dora-openarm-ker --help
python -c "import dora, pyarrow, openarm_ker, dora_openarm_ker; print('imports OK')"
```

不要同时运行 `openarm-ker-cli stream` 和 Dora KER。USB Vendor 设备同一时刻只允许一个
主机进程持有。

## 7. 官方 Dora KER 最小数据流

在 `ker/dora-openarm-ker-main` 下创建测试用 dataflow 配置。该 YAML 只是连接官方节点，
不修改官方 Python 代码：

```yaml
nodes:
  - id: leader
    path: dora-openarm-ker
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - follower_position_left
      - follower_position_right
      - metadata
```

建议文件名：

```text
dataflow-ker-readonly.yaml
```

运行：

```bash
cd /home/alientek/openflex/ker/dora-openarm-ker-main
source .venv/bin/activate
dora run dataflow-ker-readonly.yaml
```

预期日志：

```text
Connecting to KER device...
=== Verified Device Metadata ===
Hardware : ...
Firmware : ...
Updated  : ...
KER Leader Node Running successfully.
```

该节点依赖 `tick` 才读取并输出数据。初次验证不启用 `--hampel`，以便直接观察固件原始
角度是否有跳变；基础信号通过后才评估滤波。

若所安装的 Dora CLI 支持 topic 调试，可先查看：

```bash
dora topic --help
```

支持时，用 debug 模式运行并观察 `leader/follower_position_right`。不同 Dora 0.5
构建的 topic 调试命令可能不同，应以本机 `dora --help` 为准。即使不能在线 echo，
节点能够完成 PING、打印 metadata 并持续运行，也可先作为 Dora 启动链路验证。

停止时按 `Ctrl+C`。官方节点会发送 STANDBY 并释放 USB。

## 8. 安装官方 OpenArm Description

建立独立工作空间，不把 OpenArmX 包加入这个工作空间：

```bash
mkdir -p /home/alientek/openflex/official_openarm_rviz_ws/src
cd /home/alientek/openflex/official_openarm_rviz_ws/src
git clone https://github.com/enactic/openarm_description.git
```

安装当前右臂 RViz 验证所需的最小依赖：

```bash
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

构建：

```bash
cd /home/alientek/openflex/official_openarm_rviz_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select openarm_description
source install/setup.zsh
ros2 pkg prefix openarm_description
```

`ros2 pkg prefix` 应返回当前 `official_openarm_rviz_ws/install` 下的路径，不能返回
OpenArmX/OpenFlex 包路径。

## 9. 启动 OpenArm 右臂 RViz

当前只有 J1-J7，没有右夹爪通道 8，因此使用官方无夹爪右臂 preset：

```bash
cd /home/alientek/openflex/official_openarm_rviz_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch openarm_description display_openarm.launch.py \
  arm_type:=v2.0 \
  robot_preset:=right_arm
```

启动后应出现：

- OpenArm v2.0 右臂模型。
- `joint_state_publisher_gui` 的 J1-J7 滑块。
- RViz 中 Fixed Frame 为 `world`。
- `/robot_description`、`/joint_states` 和 `/tf`。

检查：

```bash
ros2 topic list | grep -E 'joint_states|robot_description|tf'
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

依次移动 GUI 的 J1-J7 滑块，记录：

- 关节正方向。
- 0 rad 模型姿态。
- 各关节允许范围。
- KER 模仿该姿态时的机械可达性。

这一步验证的是官方 OpenArm 模型、关节名称和 RViz，不是 KER 实时联动。

## 10. 两条链路的对照验证

按下表逐轴对照，不需要让 Dora 连接 RViz：

| 项目 | Dora KER | OpenArm RViz |
|---|---|---|
| J1 | `qpos_right[0]` | `openarm_right_joint1` |
| J2 | `qpos_right[1]` | `openarm_right_joint2` |
| J3 | `qpos_right[2]` | `openarm_right_joint3` |
| J4 | `qpos_right[3]` | `openarm_right_joint4` |
| J5 | `qpos_right[4]` | `openarm_right_joint5` |
| J6 | `qpos_right[5]` | `openarm_right_joint6` |
| J7 | `qpos_right[6]` | `openarm_right_joint7` |

每个关节执行：

1. KER 回到参考姿态，确认 Dora qpos 接近 0 rad。
2. 缓慢正向移动该 KER 关节约 10 度。
3. 记录 Dora 对应 qpos，应约增加 `0.1745 rad`。
4. 在 RViz GUI 中将对应关节设置为 `+0.1745 rad`。
5. 比较 KER 和 OpenArm 模型的物理运动方向。
6. 若方向相反，回到固件 `invert` 和机械安装检查。

该方法能够在不新增主机适配代码的情况下确认通道顺序、单位、零点和方向。

## 11. 如果必须让 KER 实时驱动 RViz

当前约束下不能完成。最小必要适配层应当：

1. 订阅 Dora `leader/follower_position_right`。
2. 只取 `qpos[0:7]`，丢弃缺失的通道 8 夹爪。
3. 生成 `sensor_msgs/msg/JointState`：

```text
name = [openarm_right_joint1 ... openarm_right_joint7]
position = qpos[0:7]
velocity = []
effort = []
```

4. 发布到 `/joint_states`。

Dora 新版本提供 YAML ROS 2 bridge，但 bridge 只做 Arrow/ROS 消息序列化，不会把
官方 KER 的 `{qpos}` 自动改造成上述 JointState。因此仍需要一个字段转换步骤。

若后续批准该适配，建议写成一个只做格式封装的 Dora operator，并通过 Dora 官方
ROS 2 bridge 发布；不修改 `dora-openarm-ker`，也不使用我们的 KER ROS 驱动。该工作
应单独立项、测试和评审，不能把它描述为“只改 M5 标定”。

## 12. 不改主机代码的端到端替代方案

如果目标是“KER 实时驱动官方 OpenArm 仿真模型”，而不是必须使用 RViz，官方生态的
正确接收端是 `dora-openarm-mujoco`：

```text
dora-openarm-ker/follower_position_right
  -> dora-openarm-mujoco/position_right
  -> MuJoCo OpenArm viewer
```

两者都使用 `{qpos}`，接口比 RViz 更接近。但当前缺少通道 8，官方 KER 仍输出一个
无效右夹爪值；使用带夹爪的 MuJoCo 模型前应补齐通道 8，或确认 MuJoCo 支持无夹爪的
7-DOF 输入。不能默认把缺失夹爪值送入仿真。

## 13. 故障排查

### Dora 报 USB 找不到或权限不足

```bash
lsusb | grep 303a
ls -l /dev/bus/usb/*/*
```

确认 udev 规则已生效，并确保官方 CLI 已退出。

### Dora 启动后没有更新

- 确认 dataflow 给 leader 提供了 `tick`。
- 确认 M5 已进入 STREAM。
- 确认没有第二个进程占用 USB。
- 先用 `openarm-ker-cli stream` 复测固件，再退出 CLI 启动 Dora。

### RViz 模型不显示

- Fixed Frame 设置为 `world`。
- 检查 `ros2 topic echo /robot_description --once`。
- 检查 `ros2 node list` 中是否存在 `robot_state_publisher`。
- 确认使用的是官方 `openarm_description`，不是 `openarmx`。

### RViz 模型显示但不随 KER 动

在当前严格方案中这是预期行为，因为尚无 qpos -> JointState 适配层。

### J1-J7 数据正确但 Dora 仍输出左臂和夹爪

这是官方节点当前实现。它固定处理 16 通道并忽略 `errors`。不要把缺失通道输出连接到
真实 follower、controller 或仿真夹爪。

## 14. 阶段验收清单

- [ ] M5 只对通道 1-7 完成参考姿态标定。
- [ ] 没有执行 Zero All。
- [ ] 参考姿态下 `angles[0:7]` 接近 0 度。
- [ ] `errors[0:7]` 全为 False。
- [ ] 官方 CLI stream 连续运行正常。
- [ ] 官方 Dora KER 能完成 Ping、打印 metadata 并持续运行。
- [ ] 未启动 `dora-openarm` 真实 follower。
- [ ] 官方 OpenArm v2.0 `right_arm` preset 能在 RViz 正常显示。
- [ ] RViz GUI 的 J1-J7 名称、范围和正方向已记录。
- [ ] Dora qpos 与 RViz GUI 逐轴做过 0 rad 和约 10 度对照。
- [ ] 没有把 RViz 静态/GUI 验证误记为 KER 实时联动。
- [ ] 未修改官方 Dora KER 角度换算。
- [ ] 未使用 OpenArmX/OpenFlex 实机控制链路。

完成这些项目后，可以确认 M5 标定、官方 Dora KER 输出以及官方 OpenArm RViz 模型各自
正确。若下一阶段要求 KER 实时驱动 RViz，需要先批准并实现最小格式适配层。

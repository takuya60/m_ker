# OpenFlex KER 双臂下垂姿态标定手册

本文用于在没有 OpenArm KER 官方标定夹具的情况下，以 OpenFlex 日常双臂下垂姿态作为标定参考姿态，修改 KER 固件并完成编码器 Zero 标定。

## 1. 标定目标

标定流程如下：

1. 让 OpenFlex 双臂处于日常下垂姿态。
2. 确认 `/joint_states` 中左右臂 14 个关节都接近 `0 rad`。
3. 将 KER 固件中 14 个手臂通道的 `mech_joint_offset` 全部改为 `0.0f`。
4. 重新烧录 M5 固件。
5. 让 KER 模仿相同的双臂下垂姿态，然后执行 Zero All。
6. 验证 KER 的 14 个手臂关节输出是否都接近 `0 rad`。
7. 对比 KER 与 OpenFlex 的静态残差。

完成后，KER 在下垂姿态点击 Zero，其输出应直接接近 OpenFlex 下垂姿态的 `/joint_states`，ROS 配置中的 14 个 `joint_offsets` 初始保持为零。

## 2. 安全要求

标定过程中必须满足：

- 急停可用。
- 机器人周围没有人员和障碍物。
- 不启动 `openflex_ker_arm_bridge`。
- 不向左右臂 forward position controller 发布 KER 命令。
- 只读取 OpenFlex `/joint_states` 和 KER 角度。
- 首次验证前保持 `enable_safety_check: true`。
- `/ker/error_mask` 非零时禁止标定。

标定期间建议只启动 OpenFlex bringup、`joint_state_broadcaster` 和单独的 KER 驱动节点。

## 3. 固件 Zero 的工作方式

当前 KER 固件的手臂角度可以简化为：

```text
q_ker = encoder_delta_after_zero + mech_joint_offset
```

其中：

- `encoder_delta_after_zero` 是编码器相对于点击 Zero 时的角度变化量。
- 点击 Zero 的瞬间，`encoder_delta_after_zero` 为 0。
- `mech_joint_offset` 是固件赋给 Zero 姿态的坐标。

所以点击 Zero 后：

```text
q_ker = mech_joint_offset
```

OpenFlex 代码定义的双臂 `home` 是 J1-J7 全零，因此我们的修改目标是把 14 个
手臂通道的 `mech_joint_offset` 全部设为 `0.0f`。

Zero 记录的 16 路原始编码器基准保存在 M5 NVS：

```text
namespace: ker-cal
key:       offsets
```

断电重启后固件会继续使用这组编码器基准。

### 3.1 OpenFlex 全零 Home 的代码依据

当前 `openarmx_ros2` 中有三处一致定义：

- `openarmx_bimanual_moveit_config/config/openarmx_bimanual.srdf`：左右臂 `home` 的 J1-J7 全部为 `0`。
- `openarmx_bimanual_moveit_config/config/initial_positions.yaml`：仿真左右臂初始值全部为 `0`。
- `openarmx_hardware/src/v10_simple_hardware.cpp`：`return_to_zero()` 给 7 个手臂电机发送 `0.0`。

但真实硬件启动时不会自动执行 `return_to_zero()`，该调用目前已被注释。驱动会把
命令初始化为电机当前位置。因此必须在实机下垂时读取 `/joint_states`，确认电机零位
正确，而不能仅依赖配置文件。

## 4. 需要修改的固件文件

固件根目录：

```text
ker/firmware/M5
```

### 4.1 必须修改：`include/Common.h`

文件位置：

```text
ker/firmware/M5/include/Common.h
```

找到：

```cpp
const EncoderConfig ENCODER_CONFIG[NUM_SENSORS] = {
```

每一项格式为：

```cpp
// invert, mech_min, mech_max, mech_joint_offset, skip_jump_detect
```

标定时主要修改第 4 个字段 `mech_joint_offset`。

通道与关节顺序：

| 数组索引 | KER 通道 | 对应关节 | `mech_joint_offset` |
|---:|---:|---|---|
| 0 | 1 | 右 J1 | `0.0f` |
| 1 | 2 | 右 J2 | `0.0f` |
| 2 | 3 | 右 J3 | `0.0f` |
| 3 | 4 | 右 J4 | `0.0f` |
| 4 | 5 | 右 J5 | `0.0f` |
| 5 | 6 | 右 J6 | `0.0f` |
| 6 | 7 | 右 J7 | `0.0f` |
| 7 | 8 | 右夹爪 | 保持 `0.0f` |
| 8 | 9 | 左 J1 | `0.0f` |
| 9 | 10 | 左 J2 | `0.0f` |
| 10 | 11 | 左 J3 | `0.0f` |
| 11 | 12 | 左 J4 | `0.0f` |
| 12 | 13 | 左 J5 | `0.0f` |
| 13 | 14 | 左 J6 | `0.0f` |
| 14 | 15 | 左 J7 | `0.0f` |
| 15 | 16 | 左夹爪 | 保持 `0.0f` |

不要把左右臂顺序写反。KER 固件顺序是右臂在前、左臂在后。

### 4.2 必须检查：`mech_min` 和 `mech_max`

`Common.h` 同一张表中的第 2、3 个字段用于启动时的正负 360 度修正：

```cpp
mech_min
mech_max
```

写入全零下垂参考角后，必须确认 0 位于对应范围内：

```text
mech_min <= 0 <= mech_max
```

若参考角超出范围，不能只扩大数值后直接使用。应先确认：

- OpenFlex 电机零位是否正确。
- `/joint_states` 的关节名称是否取对。
- 实机下垂姿态是否确实对应代码定义的全零 Home。
- KER 与 OpenFlex 的关节定义是否一致。

只有确认坐标正确后，才根据 OpenFlex 实际关节限位调整 `mech_min` 和 `mech_max`。

### 4.3 建议修改：`include/Meta.h`

文件位置：

```text
ker/firmware/M5/include/Meta.h
```

建议更新：

```cpp
#define FW_VERSION   "2.0.0-of-cal1"
#define LAST_UPDATED "YYYY-MM-DD"
```

这样可以通过 RViz Ping 或 `/ker/metadata` 确认 M5 运行的是 OpenFlex 标定固件，避免误用官方原始固件。

### 4.4 不需要修改的文件

以下逻辑保持官方实现：

```text
ker/firmware/M5/include/AngleProcessor.h
ker/firmware/M5/src/main.cpp
```

`AngleProcessor.h` 继续负责方向处理和 0/360 度展开；`main.cpp` 继续负责 Zero 命令、单通道 Zero、NVS 保存和 Jump Detection。

## 5. 验证 OpenFlex 下垂姿态确实为全零

### 5.1 准备机器人

1. 启动 OpenFlex bringup。
2. 不启动任何 KER 控制节点。
3. 让双臂进入日常下垂待机姿态。
4. 确认肘部、腕部和夹爪方向与以后标定时一致。
5. 保持机器人静止。

“双臂下垂”不能只看末端位置。J3、J5、J7 旋转后，机械臂仍可能看起来下垂，因此建议在关节或腕部增加方向标记。

### 5.2 读取关节状态

检查话题：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

必须根据 `name` 查找位置，不能直接假设 `/joint_states.position` 的数组顺序。

需要检查：

```text
openarmx_right_joint1 ... openarmx_right_joint7
openarmx_left_joint1  ... openarmx_left_joint7
```

建议静止采集多次并取平均值，避免单帧通信噪声。验证表至少包含：

| 关节 | 平均值 rad | 与 0 的误差 deg |
|---|---:|---:|
| 右 J1 |  |  |
| 右 J2 |  |  |
| 右 J3 |  |  |
| 右 J4 |  |  |
| 右 J5 |  |  |
| 右 J6 |  |  |
| 右 J7 |  |  |
| 左 J1 |  |  |
| 左 J2 |  |  |
| 左 J3 |  |  |
| 左 J4 |  |  |
| 左 J5 |  |  |
| 左 J6 |  |  |
| 左 J7 |  |  |

### 5.3 单位转换

OpenFlex `/joint_states` 使用 rad，KER `Common.h` 使用 deg：

```text
angle_deg = angle_rad * 180 / pi
```

如果实机下垂时某个关节明显不接近 0，应先检查 OpenFlex 电机机械零位。不要为了
适配错误的机器人零位而在 KER 固件中写入一个很大的参考角。

## 6. 修改 `ENCODER_CONFIG`

先备份原始 `Common.h`。然后把 14 个手臂通道的 `mech_joint_offset` 改为 `0.0f`。

修改后的配置示意如下：

```cpp
const EncoderConfig ENCODER_CONFIG[NUM_SENSORS] = {
    // invert  mech_min  mech_max  mech_joint_offset       channel
    {false,    -80.0f,    160.0f, 0.0f, false}, // ch1  right J1
    {true,     -10.0f,    160.0f, 0.0f, false}, // ch2  right J2
    {false,    -90.0f,     90.0f, 0.0f, false}, // ch3  right J3
    {true,       0.0f,    140.0f, 0.0f, false}, // ch4  right J4
    {false,    -90.0f,     90.0f, 0.0f, false}, // ch5  right J5
    {true,     -45.0f,     45.0f, 0.0f, false}, // ch6  right J6
    {true,     -90.0f,     90.0f, 0.0f, false}, // ch7  right J7
    {false,    -90.0f,      5.73f, 0.0f, true},            // ch8  right gripper

    {false,   -160.0f,     80.0f, 0.0f, false},  // ch9  left J1
    {true,    -160.0f,     10.0f, 0.0f, false},  // ch10 left J2
    {false,    -90.0f,     90.0f, 0.0f, false},  // ch11 left J3
    {false,      0.0f,    140.0f, 0.0f, false},  // ch12 left J4
    {false,    -90.0f,     90.0f, 0.0f, false},  // ch13 left J5
    {true,     -45.0f,     45.0f, 0.0f, false},  // ch14 left J6
    {true,     -90.0f,     90.0f, 0.0f, false},  // ch15 left J7
    {false,     -5.73f,    60.0f, 0.0f, true},             // ch16 left gripper
};
```

上面仅演示需要替换的位置。`invert`、`mech_min`、`mech_max` 应以实际文件和验证结果为准，不要为了复制示例而覆盖已经确认的配置。

## 7. 编译与烧录 M5

进入固件目录：

```bash
cd ker/firmware/M5
```

USB 模式编译和烧录：

```bash
pio run -e usb --target upload
```

串口模式：

```bash
pio run -e serial --target upload
```

烧录后检查：

1. M5 正常启动。
2. 16 路编码器均可读取。
3. Jump Detection 没有立即触发。
4. Ping 返回修改后的固件版本。

```bash
ros2 service call /ker/ping std_srvs/srv/Trigger
```

## 8. 使用下垂姿态执行 Zero

1. 停止 KER ROS 驱动，使 M5 进入 STANDBY。
2. 让 KER 模仿 OpenFlex 的下垂参考姿态。
3. 对齐肩部、肘部、腕部方向标记。
4. 将两个 KER 夹爪完全张开。
5. 固定 KER，避免点击屏幕时关节移动。
6. 在 M5 上选择 `Zero Reset (All)`。
7. 确认 Zero。
8. 等待 NVS 写入完成。
9. 重启 M5，不移动 KER。

重启后，14 个手臂关节输出应全部接近 `0 deg`。

如果只更换或调整了一个编码器，可点击对应柱状通道后使用 `Confirm Zero`，不要覆盖其他通道。

## 9. 静态验证

只启动 KER 驱动：

```bash
ros2 run openflex_teleop_ker ker_driver_node
```

检查通信：

```bash
ros2 topic echo /ker/error_mask
ros2 topic echo /ker/right_arm/joint_command --once
ros2 topic echo /ker/left_arm/joint_command --once
ros2 topic echo /joint_states --once
```

此时：

- `/ker/error_mask` 应稳定为 0。
- KER 右 J1-J7 应全部接近 `0 rad`。
- KER 左 J1-J7 应全部接近 `0 rad`。
- OpenFlex 下垂姿态的左右 J1-J7 也应全部接近 `0 rad`。
- ROS 配置中的 `joint_scales` 保持 `1.0`。
- ROS 配置中的 `joint_offsets` 初始保持 `0.0`。

每个关节静态误差：

```text
error_deg = openflex_deg - ker_deg
```

建议将误差记录到表格：

| 关节 | OpenFlex deg | KER deg | 误差 deg |
|---|---:|---:|---:|
| 右 J1 |  |  |  |
| 右 J2 |  |  |  |
| 右 J3 |  |  |  |
| 右 J4 |  |  |  |
| 右 J5 |  |  |  |
| 右 J6 |  |  |  |
| 右 J7 |  |  |  |
| 左 J1 |  |  |  |
| 左 J2 |  |  |  |
| 左 J3 |  |  |  |
| 左 J4 |  |  |  |
| 左 J5 |  |  |  |
| 左 J6 |  |  |  |
| 左 J7 |  |  |  |

## 10. 单姿态精调

如果 KER 已正确摆成参考姿态，但某关节仍有小的固定误差，可修正对应的 `mech_joint_offset`：

```text
new_mech_joint_offset_deg =
    old_mech_joint_offset_deg + error_deg
```

示例：

```text
固件右 J4 参考角 = 0.00 deg
OpenFlex 右 J4    = 0.40 deg
KER 右 J4 输出    = 0.00 deg
error             = 0.40 - 0.00 = 0.40 deg

新参考角          = 0.00 + 0.40 = 0.40 deg
```

只允许用这种方式处理确认过的小静态残差。如果 OpenFlex 下垂时明显偏离全零，应先
修复 OpenFlex 电机零位，而不是把较大的误差写入 KER。

修改后：

1. 更新 `Common.h`。
2. 更新 `Meta.h` 版本或日期。
3. 重新编译并烧录。
4. 让 KER 回到同一下垂姿态。
5. 再次执行 Zero。
6. 重新进行静态比较。

不要用很大的参考角修正掩盖下列问题：

- KER 与 OpenFlex 姿态没有真正一致。
- 腕部旋转方向看错。
- 关节通道顺序错误。
- 编码器 `invert` 配置错误。
- 编码器或联轴器松动。

## 11. 夹爪标定

夹爪不直接使用 OpenFlex `/joint_states` 的 `0-0.044` 数值填写 `mech_joint_offset`，因为 KER 固件通道使用角度，上位机负责转换为 OpenFlex 夹爪开度。

官方固件执行 Zero 时，对通道 8 和 16 写入的是夹爪编码器当时的绝对原始角度；
由于两个夹爪的 `mech_joint_offset` 都是 `0.0f`，Zero 后固件发布的夹爪角度都是
`0 deg`。

官方 Dora 节点把这个角度转换为左右镜像的夹爪电机角度：

```text
右夹爪 KER 0 deg -> -60 deg -> -1.0472 rad
左夹爪 KER 0 deg -> +60 deg -> +1.0472 rad
```

OpenFlex forward position controller 不接收这种电机角度，而是接收统一的夹爪开度
关节坐标，所以我们的 ROS 适配需要转换为：

当前适配定义：

```text
OpenFlex 0.000 = 完全闭合
OpenFlex 0.044 = 完全张开
```

执行 KER Zero 时两个夹爪应完全张开。验证时分别观察：

```bash
ros2 topic echo /ker/right_arm/joint_command --once
ros2 topic echo /ker/left_arm/joint_command --once
```

完全张开应接近 `0.044`，完全闭合应接近 `0.0`。首次测试周围不要放置物体。

## 12. 常见问题

### 点击 Zero 后全部为 0 吗？

手臂会。修改固件后，KER 在下垂姿态执行 Zero，14 个手臂关节的固件输出应全部
接近 `0 deg`。两个夹爪的固件角度也是 `0 deg`，但经过 ROS 开度转换后，夹爪目标
为 `0.044`，表示完全张开。

### 出现 Jump Detected

检查：

- 是否在正确下垂姿态执行 Zero。
- 参考角是否位于 `mech_min` 和 `mech_max` 内。
- 编码器方向和通道是否正确。
- 编码器是否松动或通信异常。

不要长期关闭 Jump Detection。

### 重启后角度变化约 360 度

检查 `invert`、`mech_min`、`mech_max` 和 Zero 姿态。不要在 ROS 中添加约 `6.283 rad` 的 offset 规避。

### 只有一个关节错误

检查对应编码器 ID、方向和机械固定。若只是该通道重新安装，可使用单通道 Confirm Zero。

### 固件版本没有变化

检查 `Meta.h` 是否修改、是否烧录了正确 PlatformIO 环境，以及 USB 是否连接到目标 M5。

## 13. 启动控制前检查

启动完整 KER 遥操作前，必须确认：

- [ ] 已确认 SRDF `home` 和仿真初始姿态的左右 J1-J7 全部为 0。
- [ ] 实机双臂下垂时 `/joint_states` 的左右 J1-J7 均接近 0。
- [ ] `Common.h` 中 14 个通道顺序正确。
- [ ] 14 个手臂通道的 `mech_joint_offset` 已改为 `0.0f`。
- [ ] 0 位于各关节对应的 `mech_min/mech_max` 范围。
- [ ] `Meta.h` 能识别当前标定固件版本。
- [ ] KER 已在下垂姿态执行 Zero All。
- [ ] M5 重启后 Zero 没有丢失。
- [ ] `/ker/error_mask` 稳定为 0。
- [ ] KER 与 OpenFlex 下垂姿态静态误差符合要求。
- [ ] 两个夹爪开闭方向正确。
- [ ] 急停可用。
- [ ] `enable_safety_check` 保持为 `true`。

全部通过后才能启动：

```bash
ros2 launch openflex_teleop_ker openflex_ker_teleop.launch.py
```

第一次启动时人员远离机器人，随时准备急停，并观察控制桥打印的各关节初始差值。

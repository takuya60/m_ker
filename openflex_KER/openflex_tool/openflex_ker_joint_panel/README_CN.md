# openflex_ker_joint_panel

RViz2 实时 KER 关节角度面板。默认订阅 `/ker/joint_states`，分别显示左右臂 J1-J7 和夹爪的弧度、角度，同时显示消息频率与接收时延。

面板中的 `USB Ping` 和 `WiFi Ping` 分别调用 `/ker/ping_usb` 和
`/ker/ping_wifi`。驱动执行真实连接和官方 KER PING/schema 握手；检查当前传输时会
短暂停止并恢复推流。成功后面板显示实际端点、握手耗时、硬件版本、固件版本和
固件更新时间。USB 设备和 WiFi socket 始终只由驱动节点持有，RViz 不直接访问硬件。

WiFi 端点由 `openflex_teleop_ker/config/openflex_ker.yaml` 中的 `wifi_host` 和
`wifi_port` 决定，默认是 `openarm-ker.local:19090`。面板中的 Ping 状态与关节话题
在线状态相互独立，因此可区分“设备不可达”和“ROS 话题没有更新”。

编译并加载工作空间后，在 RViz2 中选择：

```text
Panels -> Add New Panel -> openflex_ker_joint_panel/JointAnglePanel
```

当前工作空间的完整操作：

```bash
cd /home/openflex/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  openflex_teleop_ker openflex_ker_joint_panel
source install/setup.bash
ros2 launch openflex_teleop_ker openflex_ker_rviz_sim.launch.py
```

RViz 打开后：

1. 点击顶部菜单 `Panels`。
2. 选择 `Add New Panel`。
3. 选择 `openflex_ker_joint_panel/JointAnglePanel`。
4. 默认话题保持 `/ker/joint_states`，点击 `Apply`。

面板停靠在 RViz 左侧或右侧时，Qt 的停靠布局默认只能左右调整宽度。点击面板中的
`浮动窗口` 后，可以从上下左右四个方向调整整个窗口大小；点击 `停靠窗口` 可重新停靠。
右臂与左臂表格之间还有一条可上下拖动的分割线，可单独调整两张表格的高度比例。

分终端调试时，终端 1 启动 OpenArmX + RViz，终端 2 使用以下命令同时启动 KER driver
和 bridge：

```bash
ros2 launch openflex_teleop_ker openflex_ker_right_sim_teleop.launch.py
```

如果列表中没有该插件，检查：

```bash
ros2 pkg prefix openflex_ker_joint_panel
```

然后完全关闭并重新启动 RViz。RViz 启动前必须 source 包含该插件的工作空间。

面板内容支持水平和垂直扩展，两张关节表会均分可用高度。需要注意：当 Panel 停靠在
RViz 左侧或右侧时，Qt Dock 区域本身通常只提供左右分隔条；若要直接拖动上下边界，
可将 Panel 拖成浮动窗口，或停靠到 RViz 顶部/底部。

话题输入框可切换到其他 `sensor_msgs/msg/JointState` 话题。面板同时识别
`ker_left_*`、`ker_right_*` 和 OpenFlex 的 `openarmx_left_*`、
`openarmx_right_*` 命名，因此切换到 `/joint_states` 可查看机器人实际反馈角度。
RViz 配置会保存当前话题。

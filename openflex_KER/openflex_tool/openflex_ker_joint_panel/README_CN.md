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

话题输入框可切换到其他 `sensor_msgs/msg/JointState` 话题。面板同时识别
`ker_left_*`、`ker_right_*` 和 OpenFlex 的 `openarmx_left_*`、
`openarmx_right_*` 命名，因此切换到 `/joint_states` 可查看机器人实际反馈角度。
RViz 配置会保存当前话题。

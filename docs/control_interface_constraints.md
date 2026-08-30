# 控制接口约束

本文记录 `VisionData`、ROS 话题和 `RobotCtrlData` 之间已经由电控确认的单位与安全边界。它是控制适配和串口桥的约束说明，不是正式相机标定、云台外参或开火时序的替代品。

## 参考系和单位

- `VisionData` 从下位机到上位机，`RobotCtrlData` 从上位机到下位机；两者的位置角使用同一个“上电姿态为原点”的相对控制参考系。
- `yaw`、`pitch`、`roll` 的外部单位为 `degree`；外部速度为 `degree/s`；外部加速度为 `degree/s²`。
- 算法内部位置、速度、加速度分别使用 `rad`、`rad/s`、`rad/s²`。唯一的角度转换点是 `auto_aim_ros2` 的 ROS/算法适配层；串口映射层只复制字段。
- 协议中的 C/C++ `float` 仅按 IEEE-754 binary32（fp32）记录和实现；这项类型约定不自动证明 MCU/主机的字节序、packed 对齐或线上序列化布局。即使构建使用 hard-float ABI，也不能把 ABI 选择当作已经取得线上端序或序列化证据。
- 四元数顺序是 `w,x,y,z`，记录的是相对上电原点的姿态证据。本阶段不解释旋转方向、乘法约定或 world 轴，也不用于旋转、积分、坐标变换或预测。
- PnP 的 `relative_yaw_rad`、`relative_pitch_rad` 是相机几何相对角，不能直接作为共享上电参考系中的 `RobotCtrl.yaw/pitch` 绝对目标；正式 camera→control 外参和零点仍需实测确认。

## 协议字段与帧证据边界

- `VisionData` 使用命令号 `0x0105`，payload 为 47 bytes，主线接收帧边界为 58 bytes，CRC16 后带 `0D 0A`。2026-08-31 串口实采显示 MCU 发送 `data_length=46`（vision 结构体去掉末尾 `game_progress` 字节），接收侧同时接受 46/47。
- `RobotCtrlData` 使用命令号 `0x0102`，payload 为 26 bytes，主线下发帧边界为 37 bytes，CRC16 后带 `0D 0A`。2026-08-31 实采显示 MCU 自身所有帧均带 `0D 0A`，与 DJI 帧格式一致；此前 PR #13 的 35-byte 无尾假设已被该证据推翻。
- CRC8/CRC16 的当前软件回归约定分别使用初值 `0xFF`/`0xFFFF`；CRC16 写入顺序为低字节在前、高字节在后。软件向量和 loopback 只证明本地实现，不能被写成 MCU raw-hex golden frame。
- `bullet_count` 仅记录累计发送次数，不代表剩余弹量，也不参与目标、弹道或开火决策；`game_progress` 保留为历史协议字段并只记录，不参与算法。
- `fire_command` 在当前所有离线、dry-run 和安全控制路径必须保持 `0`；在电控确认 `1/2` 的脉冲/电平、保持时间和停止规则前，不得发送非零值。

## 车型 profile 和位置约束

控制输出必须显式选择车型 profile：

| profile | pitch 预限幅（degree） |
|---|---:|
| `new_turtle` | `[-20, 19]` |
| `dog_leg` | `[-10, 31]` |

`unselected` 或非法 profile 不假定通用范围，直接 fail-closed：保持位置安全值、`target_lock=50`、四个运动字段为 `0`、`fire_command=0`。选定 profile 后，视觉侧会把 pitch 预限幅并记录 `pitch_clamped` 诊断；下位机仍是最终机械 pitch 限位层。

Yaw 在输出边界归一化到闭区间 `[-180, 180]` degree。NaN、Inf、非法枚举和不完整命令不会被归一化成有效控制。

## 发送和超时

`RobotCtrlSub` 的 `output_hz` 默认 `100` Hz，使用可配置的纳秒周期，允许根据实测提高到数百 Hz。视觉侧已有 `input_timeout_ms=100` 的安全策略：输入过期时保持最近有效 yaw/pitch，解锁并清零速度、加速度和开火。这个 100 ms 是本机视觉/桥接保护，不等同于 MCU watchdog；MCU watchdog 约 500 ms 且可能因车型不同，必须以电控实测为准。

## 前馈、开火和硬件边界

- 外部速度/加速度单位已经确认，但 MCU 对四个前馈字段的控制语义仍未确认，因此 ROS/串口真实控制路径始终发送 `yaw_vel=yaw_acc=pitch_vel=pitch_acc=0`。
- 当前控制接口阶段无论上游请求什么，安全层都将 `fire_command` 固定为 `0`；`1`/`2` 的脉冲、电平、保持时间和停止规则仍待确认。
- `AutoAimNode` 在启动时拒绝 `allow_fire=true`；`RobotCtrlSub` 即使收到该参数或直接 ROS 输入，也会在最终安全边界抑制开火。当前不存在可审核的 production fire profile。
- 默认 `dry_run=true`、`serial_enabled=false`、`allow_fire=false`。dry-run、离线测试和 ROS 集成测试不连接真实串口，不运动云台，不发射。
- `protocol_new.hpp` 的字段、packed 布局、CRC、命令号、端序和帧解析不在本约束层修改。

## 仍待确认

通信链路确认是 **CDC USB**，因此本链路不应被描述为需要统一 UART 波特率；代码中的 `115200` 仅保留为历史兼容参数，不能写成已确认的物理波特率或 USB line coding。2026-08-31 已从 `/dev/robomaster` 抓取到 MCU→主机方向完整原始帧（160 bytes，含 2 帧完整 VISION 帧，CRC8/CRC16 与仓库实现逐字节一致，均带 `0D 0A`）；主机→MCU 方向的 golden frame 仍未取得，电控接收端对帧尾与 `data_length` 语义的确认仍待补。

仍待确认的项目包括：主机→MCU 方向 golden frame、MCU/Orin 端序与 FP32 ABI/packing 的现场一致性、四元数旋转方向、`mode=33` 是否为控制前提、各车型精确 watchdog 和最终限位、正式相机 K/D、装甲尺寸、camera→control 外参，以及开火时序。上述事项未闭环前不得猜测或扩大硬件联调范围。

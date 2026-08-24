# 控制接口约束

本文记录 `VisionData`、ROS 话题和 `RobotCtrlData` 之间已经由电控确认的单位与安全边界。它是控制适配和串口桥的约束说明，不是正式相机标定、云台外参或开火时序的替代品。

## 参考系和单位

- `VisionData` 从下位机到上位机，`RobotCtrlData` 从上位机到下位机；两者的位置角使用同一个“上电姿态为原点”的相对控制参考系。
- `yaw`、`pitch`、`roll` 的外部单位为 `degree`；外部速度为 `degree/s`；外部加速度为 `degree/s²`。
- 算法内部位置、速度、加速度分别使用 `rad`、`rad/s`、`rad/s²`。唯一的角度转换点是 `auto_aim_ros2` 的 ROS/算法适配层；串口映射层只复制字段。
- 四元数顺序是 `w,x,y,z`，记录的是相对上电原点的姿态证据。本阶段不解释旋转方向、乘法约定或 world 轴，也不用于旋转、积分、坐标变换或预测。
- PnP 的 `relative_yaw_rad`、`relative_pitch_rad` 是相机几何相对角，不能直接作为共享上电参考系中的 `RobotCtrl.yaw/pitch` 绝对目标；正式 camera→control 外参和零点仍需实测确认。

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
- 默认 `dry_run=true`、`serial_enabled=false`、`allow_fire=false`。dry-run、离线测试和 ROS 集成测试不连接真实串口，不运动云台，不发射。
- `protocol_new.hpp` 的字段、packed 布局、CRC、命令号、端序和帧解析不在本约束层修改。

## 仍待确认

波特率、真实 golden frame、MCU/Orin 端序和 FP32 ABI、四元数旋转方向、`mode=33` 是否为控制前提、各车型精确 watchdog 和最终限位、正式相机 K/D、装甲尺寸、camera→control 外参，以及开火时序仍不得猜测。

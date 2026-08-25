# 离线 Aimer 输出和单位边界

## 单位

| 字段 | 单位 | 说明 |
|---|---|---|
| `relative_yaw_rad` | rad | 相对角，+yaw 向左；不能直接作为 RobotCtrl 绝对角 |
| `relative_pitch_rad` | rad | 相对角，+pitch 向上；不能直接作为 RobotCtrl 绝对角 |
| `command_yaw_rad` / `command_pitch_rad` | rad | 仅 test-only absolute-zero 候选 |
| `command_yaw_degree` / `command_pitch_degree` | degree | 仅 CSV/标注诊断，来源是候选 rad 的转换 |
| `yaw_vel_rad_s` / `pitch_vel_rad_s` | rad/s | 内部诊断有限差分，不自动发送 |
| `yaw_acc_rad_s2` / `pitch_acc_rad_s2` | rad/s² | 当前为 0；不实现加速度模型 |

正式外部接口边界仍是：

```text
VisionData degree
→ Vision.msg degree
→ ROS input adapter degree → rad
→ algorithm/PnP/Aimer rad
→ ROS output adapter rad → degree
→ RobotCtrl.msg degree
→ RobotCtrlData degree
```

串口桥只做字段映射，不做角度转换。外部速度单位为 degree/s、加速度单位为 degree/s²，
但 MCU 前馈控制语义尚未确认，因此当前 ROS/串口输出适配层固定为 0。独立控制约束层负责
yaw 环绕和车型 pitch 预限幅；离线 Aimer 的 relative 输出仍不能直接作为 RobotCtrl 绝对角。

## Aimer 模式

### `relative_debug`

- 只记录 relative yaw/pitch；
- `absolute_command_valid=false`；
- 不产生正式绝对目标角；
- `fire_command=0`；
- 安全 `AimCommand` 视图保持 `target_lock=50`，防止误送入 RobotCtrl。

### `test_absolute_zero`

只有同时满足以下条件才生成候选绝对角：

- 模式明确选择为 `test_absolute_zero`；
- `test_only=true`；
- yaw/pitch 测试零点均显式配置；
- 当前轨迹为 `tracking` 且有有效 relative yaw/pitch。

候选值为 `test_zero + relative`，并始终带 `test_only=true`。这里不解释 quaternion/world，不做 camera 与云台同轴假设，不做 yaw wrap、机械限位或弹道补偿。

## 开火约束

离线 Aimer 不接 `CoreConfig.allow_fire`，所有模式的 `fire_command` 都是 0。单发/连发参数暂不进入离线链路，也不应散落到算法代码中。

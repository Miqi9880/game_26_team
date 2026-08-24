# OfflineTracker 配置说明

`TrackerConfig` 只控制离线、可解释的观测关联，不是 EKF 或正式控制器参数。

| 字段 | 类型 | 默认值 | 约束 | 含义 |
|---|---:|---:|---|---|
| `min_detect_count` | int | 2 | > 0 | 连续有效观测达到该数量才进入 `tracking` |
| `max_temp_lost_ms` | int | 100 | ≥ 0 | 最后一次有效观测后，小于该时长保持 `temp_lost`；到达边界进入 `lost` |
| `max_position_jump_m` | double | 0.75 | finite, > 0 | 同 class/armor 的相机坐标关联门限，单位 m |
| `max_angle_jump_rad` | double | 0.75 | finite, > 0 | 相邻 relative yaw/pitch 的拒绝门限，单位 rad |
| `max_velocity_rad_s` | double | 0 | finite, ≥ 0 | 可选有限差分诊断门限；0 表示关闭，不是预测模型 |

## 时间要求

离线工具使用录像帧时间 `stamp_ns`，不得使用不可复现的 `steady_clock::now()`。帧时间必须严格递增，`TargetObservation.stamp_ns` 必须与 update 的帧时间相同。回退、重复或负时间戳拒绝更新，并禁止本次输出锁定。

## 坐标和有效性

位置跳变比较始终使用 camera xyz，避免把 camera frame 和 gimbal frame 混合比较。有效观测要求：

- `valid=true`、`geometry_known=true`；
- class、置信度、bbox、四点和重投影误差有限；
- camera xyz 有限且 `z > 0`；
- relative angle 若存在则必须有限。

未知 class 或未知装甲几何应在 PnP 阶段失败闭合，不进入 Tracker。

## 状态和锁定

| 状态 | 含义 | `target_lock` |
|---|---|---:|
| `lost` | 无有效活动轨迹或超时 | 50 |
| `detecting` | 已有有效观测但未达到连续数量 | 50 |
| `tracking` | 达到连续数量且当前观测有效 | 49 |
| `temp_lost` | 短时未观测，保留轨迹诊断 | 50 |

Track 只做相邻观测的有限差分速度，首帧速度为 0；不做未经标定验证的 EKF、未来位置预测、角度 wrap 或弹道补偿。

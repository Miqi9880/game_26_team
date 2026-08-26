# OfflineTracker 配置说明

`TrackerConfig` 只控制离线、可解释的观测关联，不是 EKF 或正式控制器参数。下面的默认值和
测试 fixture 只用于合成回放保护，不能被宣称为正式比赛参数。

| 字段 | 类型 | 默认值 | 约束 | 含义 |
|---|---:|---:|---|---|
| `min_detect_count` | int | 2 | > 0 | 连续有效观测达到该数量才进入 `tracking` |
| `max_temp_lost_ms` | int | 100 | ≥ 0 | 最后一次有效观测后，小于该时长保持 `temp_lost`；到达边界进入 `lost` |
| `max_position_jump_m` | double | 0.75 | finite, > 0 | 同 class/armor 的相机坐标关联门限，单位 m |
| `max_angle_jump_rad` | double | 0.75 | finite, > 0 | 相邻 relative yaw/pitch 的拒绝门限，单位 rad |
| `max_velocity_rad_s` | double | 0 | finite, ≥ 0 | 可选有限差分诊断门限；0 表示关闭，不是预测模型 |

## 时间要求

离线工具使用录像帧时间 `stamp_ns`，不得使用不可复现的 `steady_clock::now()`。帧时间必须严格递增，`TargetObservation.stamp_ns` 必须与 update 的帧时间相同。回退、重复或负时间戳拒绝更新，并禁止本次输出锁定。回放 fixture 可以携带 frame 编号来标出数据空洞，但 Tracker 不补帧、不从编号推导时间、不跨 topic/device/clock domain 比较时间。

## 坐标和有效性

位置跳变比较始终使用 camera xyz，避免把 camera frame 和 gimbal frame 混合比较。有效观测要求：

- `valid=true`、`geometry_known=true`；
- class、置信度、bbox、四点和重投影误差有限；
- camera xyz 有限且 `z > 0`；
- relative angle 若存在则必须有限。
- `armor_size` 必须为确认的 small/large；`RawArmorDetection.armor_type` 若不是
  `Unknown` 也必须是确认的 small/large 且与之相符。`Unknown` 只可保留在 PnP 已给出合法
  显式 `armor_size` 的观测中，不能被 Tracker 映射成某一种装甲尺寸。

未知 class、不支持的 armor enum 或冲突的 armor 语义应在 PnP/Tracker 边界失败闭合，不进入可锁定轨迹。

## 状态和锁定

| 状态 | 含义 | `target_lock` |
|---|---|---:|
| `lost` | 无有效活动轨迹或超时 | 50 |
| `detecting` | 已有有效观测但未达到连续数量 | 50 |
| `tracking` | 达到连续数量且当前观测有效 | 49 |
| `temp_lost` | 短时未观测，保留轨迹诊断 | 50 |

Track 只做相邻观测的有限差分速度，首帧速度为 0；不做未经标定验证的 EKF、未来位置预测、角度 wrap 或弹道补偿。可关联边同时受 identity、camera xyz、角度和可选速度门限约束，并以距离、`track_id` 和稳定观测顺序作为偏好，通过增广路径保证最大基数；输入 detection vector 的排列不应改变结果。有效观测因边竞争而未分配时记录为 association conflict，不计入结构性 invalid。

关联失败保留旧轨迹诊断。`max_temp_lost_ms` 的边界语义为“到达该时长即 `lost`”；在此之前为 `temp_lost`，但两者都不会产生 `target_lock=49`。`temp_lost` 内重新关联会重新经过 `min_detect_count` 连续帧门槛；达到超时后的新观测不会复用旧轨迹。

## TargetSelector 配置

`TargetSelectorConfig` 也只适用于离线选择诊断：

| 字段 | 类型 | 默认值 | 约束 | 含义 |
|---|---:|---:|---|---|
| `confidence_tie_epsilon` | float | `1e-6` | finite, ≥ 0 | 两个置信度被视为近似相等的范围 |
| `switch_debounce_frames` | int | 1 | > 0 | 替代目标连续胜出的选择次数；1 表示立即切换 |

Selector 永远只返回有效 `tracking` 轨迹。防抖只在上一目标仍是有效 `tracking` 候选时保留它；旧目标变成 `temp_lost`、`lost` 或无效时立即丢弃，绝不为了稳定性返回陈旧目标。防抖计数作用于连续 selector 调用，离线调用方应每帧调用一次；诊断会记录调用数、候选数、切换数、防抖保留数和切换原因。这些统计不是实车稳定性或命中率。

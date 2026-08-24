# 离线自瞄链路设计

## 范围

本模块只用于旧 YOLOv5 参考模型、录像和 test-only PnP 配置的离线验证。它不创建 ROS publisher，不打开串口，不连接真实相机/云台，也不产生开火请求。

```text
OpenVinoYoloDetector
  → RawArmorDetection
  → PnpStage
  → TargetObservation
  → OfflineTracker
  → TargetSelector
  → SafeOfflineAimer
  → CSV + PNG
```

离线模块独立于 `auto_aim_core` 的简化 `Detection` 接口。后者仍用于 ROS dry-run，不应把 PnP 相对角塞入其中充当正式绝对角。

## 数据边界和单位

- `RawArmorDetection`：图像像素证据，包含类别、颜色、置信度、bbox 和四点，不包含 yaw/pitch 或 fire command。
- `PoseObservation`：PnP 测量结果。相机坐标单位为 m，OpenCV 相机系为 x 右、y 下、z 前。
- `TargetObservation`：补充帧时间戳和 detection index；相机坐标用于 track 关联。gimbal 坐标和 relative angle 只有经过验证的外参存在时才有。
- `relative_yaw_rad`：+yaw 向左；`relative_pitch_rad`：+pitch 向上。二者只是相对几何量，不能直接写入 `RobotCtrl.yaw/pitch`。
- 算法内部角度为 rad、速度为 rad/s、加速度为 rad/s²；诊断 CSV 可额外输出 degree。正式 ROS/串口输出由唯一适配层处理，外部速度/加速度单位虽已确认，因 MCU 前馈语义未确认仍保持 0。

## Tracker

Tracker 维护单调递增的 `track_id`，按 `class_id + armor_size` 和相机坐标距离进行贪心关联。每个帧必须提供严格递增的 `stamp_ns`，并且观测自己的时间戳必须与帧时间一致。

状态机：

```text
lost → detecting → tracking
tracking → temp_lost → lost
```

只有 `state=tracking` 且观测有效时才允许 `target_lock=49`；其他状态均为 50。时间倒退、重复时间戳、NaN/Inf、非正深度、身份不符、位置/角度跳变都会 fail-closed。速度只做相邻相对角的有限差分，不做 EKF、弹道预测、yaw wrap 或 world/IMU 补偿。

## TargetSelector

只从有效的 `Tracking` track 中选择目标，规则固定为：

1. confidence 最高；
2. 置信度近似相等时优先上一帧 `track_id`；
3. 再按 bbox 中心到图像中心的距离；
4. 最后按 `track_id` 做确定性排序。

不根据 `id=7/107` 或颜色字段猜敌我，也不把未知类别强行映射为装甲尺寸。

## SafeOfflineAimer

`relative_debug` 只输出 relative rad，不生成 absolute command。`test_absolute_zero` 只有显式 test-only yaw/pitch 零点时才计算：

```text
candidate_yaw_rad   = test_zero_yaw_rad   + relative_yaw_rad
candidate_pitch_rad = test_zero_pitch_rad + relative_pitch_rad
```

候选角度始终标记 `test_only=true`，不能进入 ROS 或串口。`fire_command` 永远为 0；内部速度仅作诊断，RobotCtrl 速度和加速度不能由此自动发送。

`SafeOfflineAimer` 接收 `shoot_speed_mps` 并原样保留为诊断字段；本阶段不进行弹道补偿、提前量或开火决策。离线录像没有对应 VisionData 时显式传入 0。

## 离线工具输出

`auto_aim_offline` 要求显式提供版本化 `--model-profile`；读取 test-only 模型时还必须传入
`--allow-test-profile`，读取 test-only PnP 时传入 `--allow-test-config`。它输出逐帧 CSV 和标注 PNG。
CSV 记录检测数量、有效 PnP 数量、相机坐标、relative rad、track 状态、track id、lock、可选
test absolute rad/degree、fire 和 test-only 标志。标注图叠加四点、PnP 状态、selected track、
tracking 状态、相对角和 `fire=0`。

录像没有有效 FPS 时必须通过 `--fps` 或 `--frame-period-ms` 提供可复现时间；不会把所有帧时间戳写成 0。

## 仍不是比赛结论

test-only 内参、装甲尺寸和外参只证明软件链路可以运行，不证明真实距离、角度或命中率。正式模型语义、相机标定、装甲实测尺寸、camera→gimbal 外参、绝对角零点和开火时序仍需单独确认。

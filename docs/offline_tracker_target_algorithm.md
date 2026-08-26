# OfflineTracker / TargetSelector 算法与回放说明

## 结论边界

本文件只说明 `auto_aim_ros2` 的确定性离线轨迹管理与目标选择。实现接受已经通过 PnP 的
`TargetObservation`，不打开相机、ROS topic、串口、云台或开火机构。所有运行仍保持：

```yaml
serial_enabled: false
dry_run: true
allow_fire: false
fire_command: 0
yaw_vel: 0
pitch_vel: 0
yaw_acc: 0
pitch_acc: 0
production_ready: false
hardware_validation: false
gimbal_closed_loop_validated: false
firing_validated: false
real_hit_rate_computed: false
```

`target_lock=49` 仅表示一条有效离线轨迹进入 `tracking`；它不是云台锁定、命中率或实车稳定性结论。

## 输入和时间边界

- 关联坐标始终是 PnP 的 camera xyz（m）；不混用 gimbal xyz，不推导 world 坐标，也不使用 quaternion。
- 每个 `TargetObservation.stamp_ns` 必须等于当前帧的 `stamp_ns`，且帧时间严格递增。负值、重复或回退时间戳返回 fail-closed 诊断，不能锁定。
- 观测必须具有有限的 class、置信度、bbox、四个关键点、重投影误差、camera xyz 和可选 relative angle；camera z 必须为正。
- 身份只由确认的 `class_id + armor_size` 限制。未知类别、不支持的 armor enum 或 raw armor hint 与 PnP `armor_size` 的冲突会 fail-closed；`ArmorTypeHint::Unknown` 只有在 PnP 已给出合法显式大小时才可保留，绝不被强制映射为 small 或 large。
- 没有补帧。回放 fixture 中的 frame 编号缺口只是数据空洞诊断；状态演进只使用输入 `stamp_ns`。

## 多目标关联与轨迹状态

每帧先验证观测，再用稳定的物理证据顺序处理候选。只有同时满足下列条件的轨迹—观测边才可关联：

1. 相同 `class_id + armor_size`；
2. 两端均为有效、非 `lost` 的 camera-frame 证据；
3. camera xyz 距离不超过 `max_position_jump_m`；
4. 可用 relative yaw/pitch 的差不超过 `max_angle_jump_rad`；
5. 可选有限差分速度不超过 `max_velocity_rad_s`。

可用边按距离、再按 `track_id` 和稳定观测顺序作为确定性偏好，并用增广路径求最大基数匹配。这样 detector vector 的排列不会改变创建、关联或选择结果；失败边不会自动创建可锁定的错误轨迹。结构无效观测与有效观测的竞争冲突分别记录为 `rejected_invalid` 与 `rejected_association_conflict`。关联失败时保留旧轨迹，用 `temp_lost` 或 `lost` 状态以及关联原因供诊断。

状态机保持为：

```text
lost → detecting → tracking
tracking → temp_lost → lost
```

连续有效观测达到 `min_detect_count` 才能进入 `tracking`。短遮挡时轨迹保留为 `temp_lost`，但绝不产生
`target_lock=49`。最后有效观测到当前帧的间隔达到 `max_temp_lost_ms` 时进入 `lost`；之后再次出现的证据会建立新的捕获边界，不能静默复用无关 track id。`temp_lost` 内重新关联时重启连续帧门槛，避免单帧重现立即锁定。

位置、角度和速度门限是当前合成 fixture 的离线保护阈值，不是正式比赛参数、更不是 EKF 协方差或运动模型。

## 可解释诊断

Tracker 每帧和每条轨迹都保留确定性的关联结果/原因（如 `new_track`、`matched`、`reacquired`、`missed`、`expired` 或拒绝原因），并累积统计：有效/接受/拒绝观测、重捕获、丢失、超时、时间戳拒绝和跳变拒绝数量。统计仅描述本次输入序列的处理路径；它不估计真实车辆速度、距离误差或命中概率。

`offline_tracker_replay_test` 使用真实 C++ `OfflineTracker` 和 `TargetSelector`，而不是 Python 的算法副本。其合成序列覆盖连续运动、多目标乱序、近距离交叉、遮挡、超时、重新捕获、时间异常、非有限数据、非正深度、跳变和确定性重复回放。CTest XML 与源码中的 fixture 一起构成可复现的离线回放证据；该测试不生成或伪造真实录像/模型证据包。

## TargetSelector

Selector 只从有效且处于 `tracking` 的轨迹中选择。候选排序不使用敌我颜色、固定 class id、车辆编号、前哨站规则或装甲专属优先级：

1. confidence 高者优先；
2. 在 `confidence_tie_epsilon` 内，上一帧的 `track_id` 优先；
3. 再按 bbox 中心到当前图像中心的距离；
4. 最后按较小的 `track_id`。

可选 `switch_debounce_frames` 只在上一目标仍是有效 `tracking` 候选时延迟替代目标；新候选需在连续 selector 调用中胜出相应次数（离线调用方应每帧调用一次）。若旧目标不存在、失效或变为 `temp_lost/lost`，selector 不会为了防抖保留它，而是立即选择当前有效候选，或返回空。切换次数、候选数量、debounce hold 与替换原因均为诊断字段；`selection_count` 统计 selector 调用次数。

## 同济代码参考边界

只读查看了：

- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/tracker.hpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/tracker.cpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/target.hpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/target.cpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/aimer.hpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/aimer.cpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/solver.hpp`
- `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/solver.cpp`
- `/home/ubuntu22/vision-study/sp_vision_25/LICENSE`

借鉴的是状态机中的 detecting/temp-lost 超时思想、在目标仍有效时避免频繁替换的思路，以及把发散/失锁显式记入诊断的工程习惯。本项目重新实现了 camera-frame 观测校验、身份约束、确定性关联、统计、selector 防抖和测试 fixture；没有复制同济代码，故没有引入其 MIT 代码文本或参数。

明确未采用：11 维 EKF、世界坐标、四元数方向、车辆半径/装甲高度/装甲尺寸、固定分辨率、协方差/过程噪声、颜色/前哨站/车型优先级、弹速/延迟/枪口偏置、Planner/MPC/Shooter、串口和完整 main 循环。正式模型、K/D、实测装甲尺寸、camera→gimbal 外参、绝对零点、Orin、真实相机/串口/云台/开火验证仍未完成。

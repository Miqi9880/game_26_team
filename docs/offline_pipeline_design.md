# 离线自瞄链路设计

## 范围

本模块只用于旧 YOLOv5 参考模型、录像和 test-only PnP 配置的离线验证。它不创建 ROS publisher，不打开串口，不连接真实相机/云台，也不产生开火请求。未来生产 OpenVINO IR 只能通过 schema-v2 的显式 XML/BIN manifest 接入：两个路径和两个 SHA-256 都必须先验证，加载时显式传入 XML 与 BIN；本模块不会从 XML 文件名推测权重文件。

```text
OpenVinoYoloDetector
  → RawArmorDetection
  → PnpStage
  → TargetObservation
  → OfflineTracker
  → TargetSelector
  ├─→ SafeOfflineAimer (unchanged safe output)
  ├─→ OfflinePredictor (optional test-only diagnostic)
  └─→ OfflineBallisticDiagnostic (optional test-only diagnostic)
       → CSV + PNG evidence
```

离线模块独立于 `auto_aim_core` 的简化 `Detection` 接口。后者仍用于 ROS dry-run，不应把 PnP 相对角塞入其中充当正式绝对角。

## 数据边界和单位

- `RawArmorDetection`：图像像素证据，包含类别、颜色、置信度、bbox 和四点，不包含 yaw/pitch 或 fire command。
- `PoseObservation`：PnP 测量结果。相机坐标单位为 m，OpenCV 相机系为 x 右、y 下、z 前。
- `TargetObservation`：补充帧时间戳和 detection index；相机坐标用于 track 关联。gimbal 坐标和 relative angle 只有经过验证的外参存在时才有。
- `relative_yaw_rad`：+yaw 向左；`relative_pitch_rad`：+pitch 向上。二者只是相对几何量，不能直接写入 `RobotCtrl.yaw/pitch`。
- 算法内部角度为 rad、速度为 rad/s、加速度为 rad/s²；诊断 CSV 可额外输出 degree。正式 ROS/串口输出由唯一适配层处理，外部速度/加速度单位虽已确认，因 MCU 前馈语义未确认仍保持 0。

## Tracker

Tracker 维护单调递增的 `track_id`，只在相同 `class_id + armor_size` 的有效观测之间关联。关联坐标始终是 camera xyz；gimbal xyz、quaternion 和世界坐标不会参与比较。候选边必须通过 camera 位置、relative angle 和可选有限差分速度门限；匹配以稳定边顺序作为偏好，并用增广路径求最大基数，再由 `track_id` 和稳定观测顺序消除等价选择，因此 detection 输入排列变化不会改变轨迹结果。失败边不创建可锁定的替代轨迹，而是保留旧轨迹诊断。

每个帧必须提供严格递增的 `stamp_ns`，并且观测自己的时间戳必须与帧时间一致。回放中的 frame 编号缺口只记录数据空洞；Tracker 不补帧，也不以不可复现的时钟、跨设备时间或 cross-topic 时间作补偿。

状态机：

```text
lost → detecting → tracking
tracking → temp_lost → lost
```

只有 `state=tracking` 且观测通过完整有效性校验时才允许 `target_lock=49`；其他状态均为 50。时间倒退、重复时间戳、NaN/Inf、非正深度、不支持或冲突的装甲语义、身份不符、位置/角度跳变都会 fail-closed。`RawArmorDetection::ArmorTypeHint::Unknown` 仅在 PnP 已给出合法、显式 `armor_size` 时可继续作为该 PnP 结果的原始提示，绝不把未知值强行映射为 small/large。速度只做相邻相对角的有限差分，不做 EKF、弹道预测、yaw wrap 或 world/IMU 补偿。

短遮挡只进入 `temp_lost`，其中绝不锁定；重新出现后重新经过连续帧门槛。到达 `max_temp_lost_ms` 边界即进入 `lost`，之后不能复用旧轨迹给无关观测。结构无效与关联竞争分别记录为 `rejected_invalid` 和 `rejected_association_conflict`。TrackerUpdate 和每条轨迹均保留关联结果/原因，以及接受、拒绝、重捕获、miss、超时和时间戳拒绝统计，供离线回放解释而非性能宣称。

## TargetSelector

只从有效的 `Tracking` track 中选择目标，规则固定为：

1. 先求候选中的全局最高 confidence；
2. 取与最高值之差不超过 `confidence_tie_epsilon` 的固定集合，并优先保留上一目标；
3. 再按 bbox 中心到图像中心的距离；
4. 最后按 `track_id` 做确定性排序。不使用逐对 epsilon 比较，避免非传递关系造成输入顺序依赖。

可选 `switch_debounce_frames` 仅在上一目标仍是有效 `Tracking` 候选时，要求替代目标在连续的 selector 调用中胜出相应次数；如果上一目标失效、`temp_lost` 或 `lost`，selector 立即放弃它，绝不因为防抖返回陈旧目标。每次调用（包括无候选的安全返回）记录候选数、切换/防抖次数和原因。当前 API 没有额外 frame token，因此离线调用方应每帧调用一次 selector。

不根据 `id=7/107` 或颜色字段猜敌我，也不把未知类别强行映射为装甲尺寸。

## SafeOfflineAimer

`relative_debug` 只输出 relative rad，不生成 absolute command。`test_absolute_zero` 只有显式 test-only yaw/pitch 零点时才计算：

```text
candidate_yaw_rad   = test_zero_yaw_rad   + relative_yaw_rad
candidate_pitch_rad = test_zero_pitch_rad + relative_pitch_rad
```

候选角度始终标记 `test_only=true`，不能进入 ROS 或串口。`fire_command` 永远为 0；内部速度仅作诊断，RobotCtrl 速度和加速度不能由此自动发送。

`SafeOfflineAimer` 接收 `shoot_speed_mps` 并原样保留为诊断字段；本阶段不进行弹道补偿、提前量或开火决策。离线录像没有对应 VisionData 时显式传入 0。

## OfflinePredictor：恒速基线与延迟诊断

`OfflinePredictor` 是独立的、默认关闭的回放诊断组件。只有显式传入
`--prediction-horizon-ms` 才会启用；显式传入 `0` 仍表示一次有意的零 horizon 诊断。
`--max-prediction-horizon-ms` 设置上限（默认 500 ms），超过上限直接 fail-closed，不能静默截断。
CLI 将毫秒四舍五入为整数纳秒；预测器只使用帧的 `stamp_ns`，不读取
`steady_clock::now()` 或其他墙钟。

对当前 selected、`state=Tracking` 且通过完整观测校验的轨迹，公式固定为：

```text
horizon_s = horizon_ns / 1e9
predicted_relative_yaw_rad = current_relative_yaw_rad + yaw_vel_rad_s * horizon_s
predicted_relative_pitch_rad = current_relative_pitch_rad + pitch_vel_rad_s * horizon_s
predicted_stamp_ns = source_stamp_ns + horizon_ns
```

角度和角速度始终是 `rad`、`rad/s`，不做 yaw wrap；degree 只在 CSV/标注中由上述 rad 派生。
负/回退/重复/不匹配时间戳、负或超限 horizon、缺少角度、NaN/Inf 角度或速度、整型时间戳溢出
以及非有限结果都有单独的 `PredictionFailureReason`。`Lost`、`TempLost`、`Detecting`、无目标或
无效轨迹均不能预测。预测结果固定 `test_only=true`、`production_ready=false`，不会回写
`TargetSelector`，不会替换 `SafeOfflineAimer` 的 selected track，也不会进入 `RobotCtrl`。

CSV 在启用时记录 prediction valid/reason、horizon、源/预测时间戳、relative yaw/pitch rad 及
派生 degree；默认关闭时 CSV 显式记录 `prediction_valid=0`、`prediction_reason=disabled` 和
不可变的 test-only/production-ready 标志，时间/角度字段保持空，以免把未启用诊断误写成零 horizon。
标注只叠加 test-only 文字，不改变控制输出。
`diagnose_synthetic_prediction_error` 只比较合成回放中预测时间戳与未来测量角，结果标记
`synthetic=true`，不等于真实延迟、命中率或比赛性能。

## OfflineBallisticDiagnostic：无阻力低弹道与 horizon 联合诊断

`OfflineBallisticDiagnostic` 是与 `OfflinePredictor` 并列的、默认关闭的纯离线分支。它不写回
`TargetSelector`、`SafeOfflineAimer`、`AimCommand`、ROS 或 `RobotCtrl`，不创建 publisher，也不打开
相机/串口。即使解析解有效，输出也固定为 `test_only=true`、`production_ready=false`、
`ballistic_control_applied=false`。

核心 `OfflineBallisticSolver` 只接受已经在**枪口弹道坐标系**中的目标点（m）：`x` 前、`y` 左、`z` 上。
不允许把 OpenCV 相机系或 gimbal 原点静默当成枪口原点。当前离线 PnP 只有 test-only
camera→gimbal 外参、没有经审查的 gimbal→muzzle 外参，因此 `auto_aim_offline` 在没有
`--allow-test-gimbal-origin-as-muzzle` 时会报告 `missing_muzzle_transform`。该显式开关只会把已有
gimbal 位置用于测试，并记录 `origin_assumption=test_only_gimbal_origin`；它不是零枪口偏移、正式外参
或生产准备结论。没有 gimbal pose 时仍 fail-closed 为 `missing_gimbal_pose`。

启用 CLI 需要同时显式提供：

```text
--ballistic-diagnostic
--ballistic-bullet-speed-mps       (> 0 m/s)
--ballistic-gravity-mps2          (> 0 m/s²)
--ballistic-system-latency-ms     (显式 0 合法)
--ballistic-max-flight-time-ms    (> 0)
```

录像没有严格时间对齐的 `VisionData`，所以不会从 `shoot_speed_mps`、latest VisionData 或任何默认值猜测
弹速/延迟/重力。所有浮点输入和坐标均检查 finite，时间都转为带溢出检查的整数 ns。

第一版只解独立推导的无空气阻力低弹道。令 `r = sqrt(x² + y²)`、弹速 `v`（m/s）、重力幅值
`g > 0`（m/s²）、高度 `z`（m），则：

```text
geometric_yaw   = atan2(y, x)
geometric_pitch = atan2(z, r)
D = v⁴ - g * (g * r² + 2 * z * v²)
tan(ballistic_pitch_low) = (v² - sqrt(D)) / (g * r)
flight_time_s = r / (v * cos(ballistic_pitch_low))
gravity_pitch_correction = ballistic_pitch_low - geometric_pitch
recommended_prediction_horizon_ns = system_latency_ns + flight_time_ns
```

`D < 0`、后方目标、过小水平距离、非法速度/重力/延迟、非有限值、飞行时间或 ns 溢出、超出显式飞行时间
上限、以及推荐 horizon 超出 `OfflinePredictor` 的上限都会分别 fail-closed。上限超出时不会截断、不会覆盖
用户的 `--prediction-horizon-ms`，也不会自动启用 Predictor。解析结果只是可解释的数学诊断，不包含空气阻力、
经验 pitch offset、枪口偏置、世界坐标、IMU、EKF、车辆模型、自动开火或实际命中率声明。

## 离线工具输出

`auto_aim_offline` 要求显式提供版本化 `--model-profile`；读取 test-only 模型时还必须传入
`--allow-test-profile`，读取 test-only PnP 时传入 `--allow-test-config`。它输出逐帧 CSV 和标注 PNG。
CSV 记录检测数量、有效 PnP 数量、相机坐标、relative rad、track 状态、track id、lock、可选
test absolute rad/degree、可选 prediction 诊断、fire 和 test-only 标志。标注图叠加四点、PnP 状态、
selected track、tracking 状态、相对角、可选 prediction/ballistic 诊断和 `fire=0`。ballistic CSV 字段
只追加在旧字段之后，包含 enabled/valid/reason、muzzle origin assumption、位置/角度、飞行时间、延迟、
推荐 horizon、速度/重力及不可变的 test-only/production-ready/control-applied 标志；还显式记录
`serial_enabled=false`、`dry_run=true`、`allow_fire=false` 与四个零速度/加速度字段。PNG 只叠加
`ballistic=<reason>` 等文字，不改变瞄点或命令。

`--video` 只接受已有的本地普通文件，拒绝 URI、相机/串口设备和 FIFO；`--csv` 不能别名 model/profile/
video/PnP 输入，也拒绝已有的非普通文件，避免离线回放误打开硬件或覆盖只读证据。`--annotated-dir`
不能别名这些输入且若已存在必须为目录。

独立 `offline_tracker_replay_test` 复用同一个 C++ Tracker/Selector，对合成序列检查逐帧
`stamp_ns`、状态、track id、选择和关联结果。它不复制算法、不依赖模型/视频/OpenVINO/ROS/相机/串口，
也不把合成回放写成真实数据证据。若未来用 `auto_aim_offline` CSV 归档真实录像回放，必须继续使用
已有 PR #16 报告器和 PR #17 evidence bundle，而不是新增 CSV 解析、哈希或准入逻辑。

录像没有有效 FPS 时必须通过 `--fps` 或 `--frame-period-ms` 提供可复现时间；不会把所有帧时间戳写成 0。

## 仍不是比赛结论

test-only 内参、装甲尺寸和外参只证明软件链路可以运行，不证明真实距离、角度或命中率。正式模型语义、相机标定、装甲实测尺寸、camera→gimbal 外参、绝对角零点和开火时序仍需单独确认。

既有 Tracker 增量只读借鉴 `/home/ubuntu22/vision-study/sp_vision_25/tasks/auto_aim/{tracker,target,aimer,solver}.{hpp,cpp}`
中的 detecting/temp-lost 状态管理、失锁诊断和避免频繁目标替换的工程思路。没有移植其 11 维 EKF、
世界坐标、四元数、车辆尺寸、固定分辨率、协方差、颜色/车型优先级、弹道、Planner/MPC/Shooter 或任何硬件代码；
也没有采用其参数作为本项目阈值。

本次 `OfflineBallisticDiagnostic` 增量没有读取或复制同济的 trajectory/aimer 源码、参数、测试数值、
弹速默认值、offset、车辆模型、EKF、控制或开火逻辑；无阻力公式、坐标约定和测试向量均在本仓库中独立写出。

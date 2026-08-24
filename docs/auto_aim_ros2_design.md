# 新版 ROS 2 自瞄主程序设计记录

## 当前范围

当前仓库没有可直接链接的新版比赛模型、正式标定资料或实机控制闭环，也没有可以直接链接的旧 `io::Command`。当前已增加独立的 OpenVINO YOLOv5 原始检测器、PnP 观测、离线 Tracker/TargetSelector/SafeOfflineAimer，以及可选的 ROS `offline_reference` dry-run backend；这些组件只用于 test-only 录像验证，不宣称真实标定精度或实机自瞄已经完成。

第一轮检测器边界如下：

```text
sensor_msgs/Image
  → ROS image adapter（拥有所有权的 BGR CV_8UC3）
  → ImageFrame
  → OpenVinoYoloDetector
  → RawArmorDetection
```

`RawArmorDetection` 只有 `class_id`、`color_id`、`confidence`、bbox 和四个图像关键点，并保留 `armor_type=Unknown`。它不包含 yaw、pitch、目标状态或 `fire_command`。

本阶段已实现的 Armor/PnP 离线边界为：

```text
RawArmorDetection
  + 当前帧相机内参/畸变（待标定资料确认）
  + 目标装甲板几何尺寸（待比赛模型确认）
  → Armor/PnP observation
  → 相机坐标或云台坐标
```

PnPStage 由独立版本化配置提供内参、畸变、装甲板尺寸与可选外参；它保留原始像素点、模型类别和置信度。它本身不发布 RobotCtrl、不使用 quaternion，也不请求开火；在 `offline_reference` dry-run 中，PnP 结果会继续进入独立的 Tracker、TargetSelector 和 SafeOfflineAimer。缺少有效 camera→gimbal 外参时，只输出 camera-frame pose；不会假定同轴或 identity 外参。完整 schema 与真实标定替换流程见 docs/pnp_config_schema.md。

## 旧仓库 API 审计结果

只读参考仓库：`/home/ubuntu22/vision-study/sp_vision_25`。

完整签名和风险记录见 `docs/legacy_auto_aim_api_audit.md`。

| 阶段 | 旧 API | 新版适配边界 |
|---|---|---|
| YOLO | `YOLO::detect(const cv::Mat &, int) -> std::list<Armor>` | `YoloStage::detect(ImageFrame) -> vector<Detection>` |
| Armor | `Armor` 多个构造函数；包含图像点、类型、世界坐标 | `ArmorStage::build(vector<Detection>) -> vector<ArmorObservation>` |
| Tracker | `Tracker::track(list<Armor>&, steady_clock::time_point) -> list<Target>` | `TrackerStage::track(vector<ArmorObservation>, time) -> optional<TargetState>` |
| Target | `Target::predict/update/ekf_x/armor_xyza_list` | `TargetState` 只暴露待确认的目标估计 |
| Aimer | `Aimer::aim(list<Target>, time, bullet_speed, bool) -> io::Command` | `AimerStage::aim(optional<Detection>, CoreConfig) -> AimCommand` |
| 串口 | 旧 `GimbalToVision/VisionToGimbal` 和 `io::Command` | ROS adapter 只映射新版 `RobotCtrl.msg` |

旧算法中的坐标变换、pitch 正方向、相机/云台外参和 firing 语义不能直接沿用；需要电控和当前相机标定资料确认。

## 当前五级内部接口

`auto_aim_core.hpp` 定义：

```text
ImageFrame → YoloStage → ArmorStage → TrackerStage
           → TargetStage → AimerStage → AimCommand
```

`AimCommand` 内部字段显式带单位：

```text
yaw_rad, yaw_vel_rad_s, yaw_acc_rad_s2
pitch_rad, pitch_vel_rad_s, pitch_acc_rad_s2
target_lock, fire_command
```

内部接口不依赖 ROS 消息、串口结构体、OpenCV 或旧 `io::Command`。

## 正式单位边界

位置角的唯一单位边界如下，任何其他节点不得重复转换：

```text
VisionData degree
  → Vision.msg degree（VisionPub 原样透传）
  → ros_adapters::to_algorithm_vision（degree → rad）
  → 算法核心 / PnP / AimCommand（rad）
  → ros_adapters::to_ros（rad → degree）
  → RobotCtrl.msg degree
  → RobotCtrlData degree（RobotCtrlSub 原样透传）
```

`yaw`、`pitch`、`roll` 的外部位置单位是 degree；核心字段明确带 `_rad`。
`yaw_vel`、`pitch_vel`、`yaw_acc`、`pitch_acc` 的外部单位尚未由电控确认，
因此 ROS 输出适配层当前强制它们为 `0`，不得把内部 rad/s 或 rad/s² 直接发送。
Vision 输入中的对应速度字段只做有限值检查，不进入算法核心。

`VisionState` 保存已转换的 Vision 角度、四元数 wxyz 和比赛记录字段，
但当前不解释 quaternion 的世界方向，不把 Vision 绝对角写入 PnP 相对角或
RobotCtrl 绝对目标角。`id`、`mode`、`bullet_count`、`game_progress` 目前只记录；
其中 `mode=33` 作为协议字段透传，不在算法中到处硬编码。

## ROS 节点

`auto_aim_node`：

- 订阅 `/image_raw`：`sensor_msgs/msg/Image`
- 订阅 `/camera_info`：`sensor_msgs/msg/CameraInfo`
- 订阅 `/Vision_data`：`auto_aim_interfaces/msg/Vision`
- 发布 `/Robot_ctrl_data`：`auto_aim_interfaces/msg/RobotCtrl`

在收到有效 `/camera_info` 前，图像回调只记录帧并产生安全命令；这避免在没有标定信息时误把图像送入真实算法。

`sensor_msgs/Image` 的 `bgr8`、`rgb8`、`bgra8`、`rgba8` 和 `mono8` 会转换为拥有所有权的 BGR `CV_8UC3`；空图、坏 `step`、数据长度不足和不支持的 encoding 会被拒绝。

默认 `backend=null` 使用 `NullYoloStage`，不会伪造真实检测结果。`backend=mock` 仅产生可观测的合成锁定结果；`backend=offline_reference` 才启用“图像→YOLO→PnP→Tracker→Selector→SafeOfflineAimer”录像链路，并要求显式模型、PnP 配置、`dry_run=true`、`serial_enabled=false` 和 `allow_test_only=true`（若配置为 test-only）。

## dry-run

默认参数：

```text
dry_run=true
allow_fire=false
mock_target=false
output_hz=100
input_timeout_ms=100
```

`dry_run=true` 时无论配置如何都强制 `fire_command=0`。`mock_target=true` 仅用于离线验证锁定和角度字段，角度由参数直接提供，不能当作真实相机坐标转换。

`shoot_speed`、`bullet_count`、`game_progress` 会经过 ROS 输入适配层保存；当前核心只保留
shoot speed 供未来弹道模块使用，bullet_count/game_progress 不参与目标、角度或开火决策。

`auto_aim_dry_run` 可以生成模拟帧，打印 CSV 到 stdout，并通过 `--csv PATH` 写出 CSV；它不初始化 ROS、相机或串口。

`auto_aim_detector_smoke` 是独立的检测器离线工具，必须显式提供 `--model` 和 `--video`。它打印模型签名和逐帧检测数量，可用 `--csv PATH` 保存原始检测，可用 `--annotated-dir PATH` 保存标注图。它始终打印 `dry_run=true allow_fire=false serial_enabled=false fire_command=0`，检测器本身不生成任何开火命令。

`auto_aim_pnp_smoke` 在检测器之后调用独立 PnpStage。它要求显式提供 `--pnp-config`；只有传入 `--allow-test-config` 才会读取 profile 为 test_only 的 YAML。它输出 RawArmorDetection 和 PoseObservation CSV、PnP 标注图、camera xyz（m）、重投影 RMS（px），以及仅在外参有效时输出的 gimbal xyz（m）和 relative yaw/pitch（rad）。它不初始化 ROS node、串口或控制发布者，并始终打印 `fire_command=0`。

`auto_aim_offline` 在上述组件之后继续运行 Tracker、TargetSelector 和 SafeOfflineAimer，输出逐帧跟踪状态、诊断锁定、相对角和 test-only 绝对角候选；它不创建 ROS publisher、不打开串口，也不产生开火请求。ROS `offline_reference` backend 使用同一条链路，但在本阶段强制 `command_publishable=false`，因此不会把候选绝对角送入 `/Robot_ctrl_data`。

当前在 WSL 中验证过的参考模型是只读旧仓库中的：

```text
/home/ubuntu22/vision-study/sp_vision_25/assets/yolov5.xml
/home/ubuntu22/vision-study/sp_vision_25/assets/yolov5.bin
```

运行时检查得到：

```text
input  = FP32 [1, 3, 640, 640]
output = FP32 [1, 25200, 22]
```

该契约、关键点重排 `[0, 3, 2, 1]`、objectness 阈值 `0.7` 和 NMS 阈值 `0.3` 都是当前参考 YOLOv5 模型的显式配置，不是对新比赛模型的假设。模型构造时会检查文件存在、输入/输出 rank、shape 和 element type；推理时再次检查输出 shape。

本阶段 smoke 命令（执行目录为 `game_26_dev`）为：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run auto_aim_ros2 auto_aim_detector_smoke -- \
  --model /home/ubuntu22/vision-study/sp_vision_25/assets/yolov5.xml \
  --video /home/ubuntu22/vision-study/sp_vision_25/assets/demo/demo.avi \
  --frames 10 \
  --csv /tmp/game26_yolov5_smoke_final.csv \
  --annotated-dir /tmp/game26_yolov5_annotated_final
```

实际结果为 10 帧处理成功，检测数量依次为：

```text
0, 1, 1, 1, 1, 1, 1, 1, 1, 1
```

CSV 和 10 张标注图均成功生成；其中一帧目视检查显示四点形成装甲板四边形。这个结果只验证参考模型的读取、前处理、输出解析和可视化，不能证明新比赛模型的类别或点序。

## 已接入但仍为离线/test-only

- `offline_reference` 已接入 ROS 节点并完成 topic dry-run；PnP → Tracker → TargetSelector → SafeOfflineAimer 的输出只用于诊断，不能作为正式绝对控制角；
- 旧参考 YOLOv5 + test-only PnP 配置已完成离线回放验证，但不能证明比赛模型、正式标定或命中效果；

## 尚未接入/待确认

- 新比赛模型的 YOLO runtime、输入输出格式、类别数和关键点顺序；参考 YOLOv5 已做离线 smoke，但不能替代新模型验证；
- Armor 识别和 `class_id → ArmorName/ArmorType` 映射；
- 新模型颜色/类别语义，以及旧模型的 class mapping 是否完全失效；
- 正式比赛相机内参、装甲物理尺寸、camera→gimbal 外参及其测量证据；当前仓库只提供 test-only 合成 PnP 配置；
- 新比赛模型的关键点语义。参考 YOLOv5 的 TL,TR,BR,BL 顺序只在 test-only smoke 中使用，循环错位无法仅靠四边形几何可靠识别，必须人工核验；
- ROS/算法适配层的 degree↔rad 边界已固定，但 Vision 绝对角的零点、RobotCtrl 绝对角的零点和二者关系仍需电控确认；
- 正式 Tracker/Target EKF 参数和运动模型；当前离线 Tracker 只做有限差分与 fail-closed 跳变检查；
- Aimer 弹道模型和 bullet speed 来源；
- target_lock/fire_command 的硬件时序；
- MCU/Orin 端序、float ABI 和 golden frame。

# 自瞄接口与标定验证记录

本文是 `game_26_dev` 的接口和标定交接记录。它描述当前已经实现的边界、可执行的实测流程和仍需队内确认的事实；不提供伪造的 production 标定数据。

## 1. 正式通信方向

```text
MCU VisionData（串口，degree）
  → VisionPub（只做协议字段映射）
  → /Vision_data : auto_aim_interfaces/msg/Vision（degree）
  → auto_aim_ros2::ros_adapters::to_algorithm_vision（degree → rad）
  → 算法核心 / PnP（rad）
  → AimCommand（rad）
  → auto_aim_ros2::ros_adapters::to_ros（rad → degree）
  → /Robot_ctrl_data : auto_aim_interfaces/msg/RobotCtrl（degree）
  → RobotCtrlSub（只做字段映射与安全策略）
  → MCU RobotCtrlData（degree）
```

串口桥不做角度单位转换。位置角字段的外部单位是 degree，速度是 degree/s，加速度是
degree/s²；算法核心字段名带 `_rad`。单位已经确认，但 MCU 前馈控制语义尚未确认，
因此输出适配层强制 `yaw_vel`、`pitch_vel`、`yaw_acc`、`pitch_acc` 为 0。Vision 输入中的
速度会转换为内部 rad/s 诊断值，但不被 Tracker、Aimer 或 RobotCtrl 使用。

`quaternion` 的顺序是 w、x、y、z，记录 IMU 相对上电原点的姿态，但 IMU→world /
world→IMU 方向、乘法约定和 world 轴仍未确认，当前只保存，不进行世界坐标补偿。`mode=33` 作为协议字段透传；`id`、`bullet_count`、
`game_progress` 只记录，不用于敌我判断、弹道或开火。

## 2.1 字段单位表

| 接口 | 字段 | 单位/含义 | 当前处理 |
|---|---|---|---|
| Vision | `id`, `mode` | 无量纲协议字段 | 原样透传和记录 |
| Vision | `yaw`, `pitch`, `roll` | degree | ROS 输入边界转换为内部 rad |
| Vision | `yaw_vel`, `pitch_vel` | degree/s | 转为 rad/s 诊断，不进入核心控制 |
| Vision | `quaternion[4]` | 无量纲，顺序 wxyz、相对上电原点 | 原样保存，不解释方向 |
| Vision | `shoot_speed` | m/s | 原样保存，暂不参与瞄准 |
| Vision | `bullet_count` | 累计发送次数 | 只记录 |
| Vision | `game_progress` | 历史内录字段 | 只记录，算法忽略 |
| RobotCtrl | `yaw`, `pitch` | degree，共享上电原点参考中的绝对目标 | 由核心 rad 转 degree，并执行 profile 约束 |
| RobotCtrl | `yaw_vel`, `pitch_vel` | degree/s | 当前固定输出 0，等待 MCU 前馈语义 |
| RobotCtrl | `yaw_acc`, `pitch_acc` | degree/s² | 当前固定输出 0，等待 MCU 前馈语义 |
| RobotCtrl | `target_lock` | 49 锁定，50 未锁定 | 原样映射并由安全策略校验 |
| RobotCtrl | `fire_command` | 0 不开火，1 连发，2 单发 | dry-run 固定为 0 |

ROS `Vision.header` 只用于时间戳和 frame_id，不进入串口结构体。

## 2. 坐标系和控制角边界

控制接口的 VisionData 和 RobotCtrlData 已确认使用同一个上电姿态原点参考。PnP 几何仍在
独立的相机坐标系中；camera→control 的正式外参尚未完成，因此不能把两者混为一谈。

PnP/相机几何的记录约定为：

```text
x：前
y：左
z：上
```

电控控制参考的上电原点和四元数轴/旋转方向仍由电控联调记录；软件不自行补偿。

软件日志约定 `+yaw` 向左、`+pitch` 向上。OpenCV PnP 相机坐标为 `x` 右、`y` 下、`z` 前。

PnP 输出的 `translation_in_camera_m` 是相机坐标；只有配置并验证
`R_gimbal_from_camera`、`t_gimbal_from_camera_m` 后，才会产生 gimbal 坐标和
`relative_yaw_rad` / `relative_pitch_rad`。这些相对角仅用于日志和几何核验，
不能直接变成 `RobotCtrl.yaw` / `RobotCtrl.pitch`，因为 camera→control 外参、安装关系和
正式标定证据尚未确认。

## 3. 标定数据边界

当前仓库没有 production PnP YAML。唯一提供的
`src/auto_aim_ros2/test/data/pnp_test_config.yaml` 是 `profile: test_only` 的合成 fixture，
包含假的内参、装甲尺寸和外参。默认加载器拒绝它；只有离线测试显式 `allow_test_only=true`
或 smoke 传入 `--allow-test-config` 才可使用。不得将 fixture 数值用于实机。

当前候选文件 `/home/ubuntu22/vision-study/game_26_dev/src/ros2-hik-camera/config/camera_info.yaml`
虽然包含 1440×1080 的 K/D 数值，但缺少相机序列号、镜头、标定日期、标定板报告和重投影误差，
只能作为格式参考，尚未提升为 production 标定来源。

正式配置必须为每一项记录来源和版本：

- 相机序列号、镜头、采集分辨率、像素格式、裁剪/缩放方式、标定日期和工具；
- `K`、`D`、重投影误差和标定板规格；
- 小/大装甲的实测宽、高、四个物理角点、单位 m 和 object frame；
- 新模型的输入尺寸、颜色顺序、预处理、输出 shape、class/color/type 语义和四点顺序；
- camera→gimbal 外参的测量方法、旋转/平移、误差、源版本和目标坐标系。

## 4. 可执行的实测验证流程

### 4.1 相机内参和畸变

1. 使用比赛实际相机、镜头和运行时分辨率采集棋盘格或 Charuco 图像。
2. 在原始 `/image_raw` 像素上标定，保存 `K`、`D`、图像尺寸、标定板规格和重投影报告。
3. 确认相机节点没有去畸变/校正；原始图像使用 raw-image `K + D`，不能直接把 `CameraInfo.P` 当作原图 K。
4. 用已知尺寸、已知距离的平面靶验证 PnP 距离和重投影误差；分辨率不一致时必须失败，不能缩放 K 猜测。

### 4.2 装甲物理尺寸和点序

1. 用卡尺或可追溯测量记录 small / large 的真实宽、高，单位统一为 m。
2. 明确 object points 顺序为 `top_left, top_right, bottom_right, bottom_left`，并让它与新模型关键点语义逐点对应。
3. 任何未知 `class_id` 且没有显式 `class_to_armor_type` 映射的检测必须拒绝 PnP，不猜尺寸。
4. 用正视、斜视、不同距离的标注图检查点序；单纯“形成四边形”不足以证明循环起点和方向正确。

### 4.3 新模型语义

1. 获取新比赛模型和导出说明，记录输入/输出 shape、FP 类型、颜色顺序和归一化方式。
2. 用 `auto_aim_detector_smoke` 检查模型签名，并对人工标注帧逐点核对 class、color、small/large 和四点顺序。
3. 只将已核验的模型契约写入版本化 `DetectorConfig`；旧 YOLOv5 的 `[0,3,2,1]` 仅是参考模型配置，不能迁移为新模型默认值。
4. 确认模型输出坐标先经过唯一的 `(model_point - padding) / scale` 反变换，再交给 PnP；不能在 PnP 内再次缩放。

历史运行 A 曾用旧仓库参考模型执行过 3 帧离线 smoke：输入为 FP32 `[1,3,640,640]`，
输出为 FP32 `[1,25200,22]`，CSV/标注输出成功；该次完整命令未随记录保存，因此不能作为
当前复现命令。历史运行 B 的 10 帧命令和结果记录在 `docs/auto_aim_ros2_design.md`，两次运行
不得合并解读。它们都只证明参考模型链路可运行，不证明新比赛模型的类别、颜色或点序。

### 4.4 camera→gimbal 外参

1. 独立测量相机到云台的旋转和平移，保存 source、version、误差和坐标轴定义。
2. 验证旋转是正交矩阵且 `det=+1`，并用已知靶检查前/左/上方向。
3. 未有真实外参时保持 `configured: false`，只输出 camera-frame pose；不得使用 identity 或旧仓库外参。

## 5. 推荐验证命令

执行目录：`/home/ubuntu22/vision-study/game_26_dev`

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select auto_aim_interfaces serical_device_ros2 auto_aim_ros2
source install/setup.bash
colcon test --packages-select auto_aim_ros2 serical_device_ros2 \
  --event-handlers console_direct+
colcon test-result --verbose
git diff --check
```

PnP 只能在获得真实配置后进行 production 验证；在此之前只能显式使用 test-only fixture：

```bash
ros2 run auto_aim_ros2 auto_aim_pnp_smoke -- \
  --model /path/to/verified_model.xml \
  --model-profile /path/to/reviewed_model_profile.yaml \
  --video /path/to/raw_image_video.avi \
  --pnp-config /path/to/production_pnp.yaml \
  --frames 10 \
  --csv /tmp/game26_pnp_validation.csv \
  --annotated-dir /tmp/game26_pnp_validation_annotated
```

该命令要求输入帧分辨率与配置完全一致，输出 camera pose、重投影误差和（仅在有效外参下）gimbal pose；它不发布 ROS 控制、不连接串口、不产生开火命令。

## 6. 电控仍需确认

1. camera→control 正式外参、机械安装关系和绝对控制零点的实测记录；
2. quaternion 是 IMU→world 还是 world→IMU，以及轴定义和乘法约定；
3. 实际波特率、端序、FP32 ABI、packed 约定和真实收发 golden frame；
4. MCU 精确频率、车型 watchdog、ACK 和断线行为；
5. 新龟/狗腿最终 pitch 机械限位与 yaw 边界的实测确认；
6. MCU 对四个前馈字段的控制语义；
7. `target_lock=49/50` 的硬件效果；
8. `fire_command=1/2` 的电平/脉冲、保持时间和停止规则；
9. `id=7/107` 的实际业务含义；
10. `mode=33` 是否是允许视觉控制的前提。

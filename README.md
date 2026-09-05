# game_26_team — 校内赛自瞄视觉（ROS2 / Humble）

蓝方车载自瞄：**传统视觉识别红色装甲板**，direct 像素伺服跟枪，串口与电控通信（协议对齐 game_26 基线）。

## 结构

```
src/
├── auto_aim_interfaces/    # 自定义消息：RobotCtrl(下行) / Vision(上行)
├── serical_device_ros2/    # 主包：auto_aim 算法节点 + 串口收发（核心工作）
└── ros2-hik-camera/        # 海康相机节点（MV-CS016-10UC, 1440x1080）
third/awakening/            # 上游算法（WUST-RM/awakening）裁剪版：detect/aim/track 核心
patches/awakening/          # 我们对上游 armor_detector 的补丁记录
start_all.sh                # 一键启动相机+算法
restart_container.sh        # 只重启算法容器
restart_hik.sh              # 只重启相机节点
```

## 算法来源与分工

- 检测/跟踪/解算算法继承自 [WUST-RM/awakening](https://github.com/WUST-RM/awakening)（`third/awakening`，裁剪保留 auto_aim 相关源码）
- 本项目增量：ROS2 工程化（节点/话题/串口/参数）、direct 像素伺服控制（bias 补偿/平滑/限速/死区）、检测过滤适配（见 `patches/`）、调试可视化
- I/O 协议对齐 [HJ-vision/game_26](https://github.com/HJ-vision/game_26)（`RobotCtrlData`/`VisionData`，921600，帧格式见 `serical_device_ros2/include/protocol.h`）

## 依赖（Ubuntu 22.04 / ROS2 Humble / Orin aarch64）

- ROS2 Humble（rclcpp / image_transport / camera_info_manager）
- OpenCV 4.10（/usr/local）、Eigen3、Ceres、yaml-cpp、spdlog
- 海康 MVS SDK：把 `libMvCameraControl.so` 等软链到
  `src/ros2-hik-camera/hikSDK/lib/arm64/`（本机路径机器相关，不入库）

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src
```

## 运行

```bash
bash start_all.sh          # 相机 + 算法全部启动（自瞄默认 aim_mode=direct）
```

关键话题：`/image_raw` `/camera_info` `/Vision_data`(上行,电控→视觉) `/Robot_ctrl_data`(下行) `/debug_image`(左上角叠加 cmd yaw/pitch + fps)

## 常用调参

```bash
ros2 param set /auto_aim direct_pitch_bias 6.1   # pitch 补偿（枪管在相机下方 13cm）
ros2 param set /hik_camera exposure_time 1200    # 曝光(整数 us)
ros2 param set /hik_camera gain 15.0             # 增益(浮点)
# 完整参数见 src/serical_device_ros2/config/auto_aim_params.yaml
```

## 补丁应用（若从完整 awakening 重新拉取）

`patches/awakening/armor_detect/armor_detector.cpp.patched` 覆盖
`third/awakening/src/tasks/auto_aim/armor_detect/armor_detector.cpp` 即可。
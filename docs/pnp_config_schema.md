# PnP 离线配置与安全边界

本文件描述 auto_aim_ros2 的 PnpStage 配置。它只建立如下离线几何链路：

    RawArmorDetection
      -> PnpStage
      -> PoseObservation（opencv_camera_optical）
      -> 可选的 gimbal_x_forward_y_left_z_up 几何变换
      -> relative yaw/pitch 日志

它不产生 RobotCtrl，不接入 Tracker、Aimer、弹道、串口或 quaternion/world 变换，也绝不请求开火。

## 坐标、单位与角度

- opencv_camera_optical：OpenCV solvePnP 相机系，+x 向右、+y 向下、+z 向前。
- gimbal_x_forward_y_left_z_up：已确认视觉云台系，+x 向前、+y 向左、+z 向上。
- camera_matrix 的 fx/fy/cx/cy 是像素量；畸变参数遵循 OpenCV 畸变模型的无量纲参数约定。
- 所有 _m 和 object_points_m 均为 metres。
- relative_yaw_rad = atan2(y_gimbal, x_gimbal)，+y_gimbal 为正（向左）。
- relative_pitch_rad = atan2(z_gimbal, hypot(x_gimbal, y_gimbal))，+z_gimbal 为正（向上）。

relative yaw/pitch 只用于日志和几何核验，不是 RobotCtrl.yaw/pitch。PnP 日志固定使用
`relative_yaw_rad` / `relative_pitch_rad`；RobotCtrl 和串口位置角固定使用 degree，
并由 ROS 适配层负责 rad→degree。绝对目标角的零点、相对角到绝对角的机械/IMU关系仍未确认，
所以 PnP 输出不能直接写入 RobotCtrl。

## YAML schema version 1

必填结构如下（数值 0.000 只是占位符，不能作为实际配置）：

    schema_version: 1
    profile: production       # 或 test_only

    camera:
      image_width: 1440
      image_height: 1080
      camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
      distortion_coefficients: [k1, k2, p1, p2, k3]
      source: calibration_report_or_fixture_identifier
      version: calibration_version
      coordinate_frame: opencv_camera_optical

    armor_geometry:
      corner_order: [top_left, top_right, bottom_right, bottom_left]
      small:
        width_m: 0.000
        height_m: 0.000
        object_points_m: [[x, y, z], [x, y, z], [x, y, z], [x, y, z]]
        source: measured_target_or_fixture_identifier
        version: geometry_version
        object_frame: named_armor_local_frame
      large:
        width_m: 0.000
        height_m: 0.000
        object_points_m: [[x, y, z], [x, y, z], [x, y, z], [x, y, z]]
        source: measured_target_or_fixture_identifier
        version: geometry_version
        object_frame: named_armor_local_frame

    # 可选；Unknown armor_type 且 class_id 未列出时拒绝 PnP，不猜尺寸。
    class_to_armor_type:
      0: small

    pnp:
      # 当前 WSL 的 OpenCV 4.1 构建显式支持 ITERATIVE，不会自动回退。
      method: ITERATIVE
      max_reprojection_error_px: 3.0

    camera_to_gimbal:
      configured: false
      # configured 为 true 时，下面字段全部必填：
      # rotation_gimbal_from_camera: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
      # translation_gimbal_from_camera_m: [tx, ty, tz]
      # source: extrinsic_calibration_report_identifier
      # version: extrinsic_calibration_version
      # source_frame: opencv_camera_optical
      # target_frame: gimbal_x_forward_y_left_z_up

已配置外参使用以下精确公式：

    p_gimbal = R_gimbal_from_camera * p_camera + t_gimbal_from_camera_m
    R_gimbal_from_armor = R_gimbal_from_camera * R_camera_from_armor

加载器会拒绝以下情况：缺失必填项、错误 schema/profile、非法尺寸、非有限数、错误角点顺序、非平面或非矩形装甲几何、无效 K/D、非正交或非 det=+1 的已配置外参、未显式支持的 PnP 方法，以及超过重投影阈值的观测。实际帧分辨率不等于 camera.image_width × image_height 时也会失败；不会缩放 K 或猜测图像是否被 resize。

## test-only 与 production

仓库中唯一提供的 PnP YAML 是：

    src/auto_aim_ros2/test/data/pnp_test_config.yaml

它是合成 fixture，使用假的相机内参、装甲尺寸和 camera->gimbal 外参。它明确声明 profile: test_only。

默认加载器会拒绝 test-only 配置。仅离线工具或测试显式设置 allow_test_only=true 或传入 --allow-test-config 后才能使用。这些数值不得复制到真实机器、正式 YAML 或控制路径。

目前没有随包提供 profile: production PnP 配置：这是有意的 fail-closed 状态，不应将当前 ros2-hik-camera/config/camera_info.yaml、旧 sp_vision_25 的 demo.yaml、旧装甲尺寸或旧外参直接迁入正式配置。

## 如何替换为真实标定

1. 用目标相机、镜头与分辨率采集棋盘格或 Charuco 数据，保存相机序列号、镜头、采集分辨率、日期、标定工具、重投影报告和版本。
2. 对原始 /image_raw 像素填写 raw-image K + D；当前相机节点未做去畸变或校正，不能把 CameraInfo.P 当作原图 K。
3. 实测小/大装甲板，明确四个物理角点与模型关键点的 top_left, top_right, bottom_right, bottom_left 对应关系；将尺寸与 object points 都以 m 记录。
4. 独立完成 camera->gimbal 外参标定，记录测量方法、版本与误差。验证其旋转映射到 x前,y左,z上，再设置 configured: true。
5. 先用 auto_aim_pnp_smoke 的 CSV、标注图和已知距离靶检查失败保护与重投影误差，再引入任何真实控制链路。

即使离线录像的重投影误差较小，也只能说明该配置对该像素/几何假设自洽，不能证明真实距离、云台角度或比赛精度。

## 当前离线命令

执行目录：/home/ubuntu22/vision-study/game_26_dev

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run auto_aim_ros2 auto_aim_pnp_smoke -- \
      --model /home/ubuntu22/vision-study/sp_vision_25/assets/yolov5.xml \
      --video /home/ubuntu22/vision-study/sp_vision_25/assets/demo/demo.avi \
      --pnp-config src/auto_aim_ros2/test/data/pnp_test_config.yaml \
      --allow-test-config \
      --frames 10 \
      --csv /tmp/game26_pnp_smoke.csv \
      --annotated-dir /tmp/game26_pnp_annotated

该命令只读旧参考模型和录像，只向 /tmp 写 CSV/PNG。启动行固定为：

    dry_run=true allow_fire=false serial_enabled=false fire_command=0

# 离线相机内参标定与证据报告工具

`auto_aim_camera_calibrate` 只读取本地图片文件中的棋盘格，生成一份
`profile: evidence_only` 的内参候选证据。它不启动 ROS 节点，不订阅图像，不打开相机，
不访问串口，也不产生云台、RobotCtrl 或开火输出。

## 用法

```bash
ros2 run auto_aim_ros2 auto_aim_camera_calibrate -- \
  --config /absolute/path/calibration_input.yaml \
  --image-list /absolute/path/images.txt \
  --report /absolute/path/camera_intrinsic_report.yaml
```

`images.txt` 每行一个图像路径；相对路径相对于 manifest 所在目录解析。工具只调用
`cv::imread`，不 resize、不裁剪、不去畸变、不猜测相机矩阵。每张图像都必须与配置的
`width_px × height_px` 完全一致。

配置示例：

```yaml
schema_version: 1
profile: evidence_only

board:
  type: chessboard
  # 内角点数量，不是方格数量
  inner_corners_cols: 9
  inner_corners_rows: 6
  square_size_m: 0.024

image:
  width_px: 1440
  height_px: 1080
  pipeline: raw_image_no_resize_no_crop_no_rectify

acceptance:
  min_accepted_views: 15
  max_global_rms_reprojection_error_px: 0.5

metadata:
  report_id: unique_identifier
  camera_serial: required_identifier
  lens_identifier: required_identifier
  acquisition_date: 2026-08-25
  operator: required_identifier
  code_commit: 7f452ef
  dataset_id: required_identifier
```

工具使用 `findChessboardCorners`、`cornerSubPix` 和 `calibrateCamera`。报告包含棋盘格
规格、分辨率、完整元数据、输入文件名、每张图像的接受/失败原因和重投影误差、全局 RMS、
候选 K/D，以及 `pnp_camera_fields_for_manual_review`。质量门禁失败（视图数量不足或 RMS
超过阈值）仍会写出 rejected evidence report，并以非零状态退出。

## 数据集 manifest 关联

旧 schema v1 配置继续兼容原命令。若 metadata 同时声明 dataset_manifest 和
dataset_manifest_sha256，必须额外传入：

~~~bash
--dataset-manifest /absolute/path/dataset_manifest.yaml
~~~

工具会在读取图像和执行标定前对 manifest 实际字节复算 SHA-256；缺少 manifest、哈希
不一致或向未声明关联的旧配置额外传入 manifest 都会直接失败。有关联时，metadata 中的相对
manifest 路径必须解析到实际传入文件，accepted records 是权威且有序的图像集；images.txt
必须逐项完全一致，每个归档 PNG 的相对路径和 SHA-256 也会在解码前复核。因此替换 image-list、
替换 PNG 或借用其他数据集都会 fail-closed。由
auto_aim_calibration_dataset 生成的 calibration_input.yaml 始终启用此验证链，标定报告
会归档已验证的 dataset_manifest_sha256。

## 安全边界

报告始终包含：

```yaml
profile: evidence_only
production_ready: false
```

它故意不是项目的 PnP YAML：手工审核字段只嵌套提供图像宽高、K、D、来源、版本和
`coordinate_frame: opencv_camera_optical`。报告不包含装甲尺寸、object points、模型类别
映射、PnP 方法、camera→gimbal 外参、绝对角、RobotCtrl 或开火字段，也不会调用
`load_pnp_configuration()`、`PnpStage` 或覆盖 `camera_info.yaml`。

因此，低 RMS 只说明这批像素和棋盘格假设在离线数学上自洽，不能证明正式相机精度。
将候选字段手工纳入正式配置前，仍必须使用实际相机、实际镜头和原始分辨率复核，并单独
完成装甲板尺寸、camera→gimbal 外参、绝对角零点及现场验证。当前任务不进行这些工作。

测试使用 `cv::projectPoints` 合成多视角角点以及临时空白/混合分辨率图像，不需要真实
图像、ROS 节点、OpenVINO、串口或任何硬件。

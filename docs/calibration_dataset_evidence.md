# 标定数据集采集、质量检查与证据归档

本流程只生成 profile: evidence_only、production_ready: false 的数据集证据。
工具不链接或探测 MVS SDK，不打开相机、串口或控制设备，不创建 ROS publisher，也不产生
/Robot_ctrl_data、云台或开火输出。manifest 固定记录 camera_sdk_status: not_used；真实相机
可用性仍由 hik_camera 的既有构建和启动诊断负责，缺少 MVS 动态库时其 CMake 会明确跳过
真实相机 executable。

## 构建与离线回归

~~~bash
cd /home/ubuntu22/vision-study/game_26_issue_26
source /opt/ros/humble/setup.bash
colcon build --packages-up-to auto_aim_tools auto_aim_ros2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select auto_aim_tools auto_aim_ros2
colcon test-result --verbose
~~~

calibration_dataset_test 使用 fixture/内存帧覆盖正常归档、空输入、空帧、混合分辨率、
错误 encoding/stride/data size、重复哈希、缺失或不匹配 CameraInfo、时间戳回退、不可读
文件、非法标定板、manifest 哈希不一致和重复输出。calibration_dataset_ros_test 用 fake
publisher 覆盖精确配对、缺 CameraInfo 超时、空输入和重复 stop；无需相机、SDK 或 Orin。

## 离线 fixture

fixture 是版本化 YAML。所有板参数和消息字段必须显式填写；不得从图片猜测标定板、时间戳
或 CameraInfo。路径相对于 fixture 所在目录解析。

~~~yaml
schema_version: 1
source:
  mode: offline_fixture
  timestamp_source: fixture_declared_ros_header
camera_info_required: false
board:
  type: chessboard
  inner_corners_cols: 9
  inner_corners_rows: 6
  square_size_m: 0.024
acceptance:
  min_views: 15
  max_global_rms_reprojection_error_px: 0.5
metadata:
  dataset_id: unique-dataset-id
  report_id: unique-report-id
  camera_serial: unknown
  lens_identifier: unknown
  acquisition_date: "2026-08-27"
  operator: operator-id
records:
  - image: input/view_0001.png
    stamp:
      sec: 1787800000
      nanosec: 100
      source: fixture:/image_raw.header.stamp
    width: 1440
    height: 1080
    encoding: rgb8
    step: 4320
    data_size: 4665600
    frame_id: camera_optical_frame
    camera_info:
      stamp:
        sec: 1787800000
        nanosec: 100
        source: fixture:/camera_info.header.stamp
      width: 1440
      height: 1080
      frame_id: camera_optical_frame
~~~

当前只支持 board.type: chessboard。其他类型会生成 rejected manifest 并返回非零。离线模式
必须显式设置 camera_info_required；未提供 CameraInfo 时，记录仍包含 present: false、
三项匹配状态和 warning。要求 CameraInfo 时，缺失或任一不匹配都是错误。

~~~bash
ros2 run auto_aim_tools auto_aim_calibration_dataset -- \
  --fixture /absolute/path/fixture.yaml \
  --output /absolute/path/dataset_run_001
~~~

输出目录必须不存在，工具拒绝覆盖或重复启动到同一路径。任何坏记录都会拒绝整个数据集，
不会静默筛除；仍可解码的帧会保留为 PNG 和逐记录诊断。只有整个数据集通过时才生成
images.txt 和 calibration_input.yaml。

## ROS 录制

ROS 配置使用同一顶层结构但不含 records，并且必须设置：

~~~yaml
schema_version: 1
source:
  mode: ros
  timestamp_source: ros_header
camera_info_required: true
# board、acceptance、metadata 与离线示例相同
~~~

~~~bash
ros2 run auto_aim_tools auto_aim_calibration_dataset_recorder -- \
  --config /absolute/path/ros_dataset_config.yaml \
  --output /absolute/path/dataset_run_002 \
  --max-frames 20 \
  --timeout-s 30 \
  --max-buffered-image-bytes 268435456
~~~

节点只订阅 /image_raw 和 /camera_info，QoS 为 SensorDataQoS。开始采集和采集过程中，两个
输入 topic 都必须恰有一个消息类型正确、best-effort/volatile 的 publisher。每个 Image 必须有
完全相同且非零规范时间戳的 CameraInfo；不使用时间容差。ROS 录制的 timestamp_source 只能是
工具可验证的 ros_header，其他硬件时间声明会在采集前拒绝。Image 必须是 packed rgb8，
step == width * 3 且 data.size() == step * height。

--max-frames 同时限制总接收并保存的 Image 数和目标配对数；达到配对数时正常停止，下一条超过
总帧上限的 Image 会在复制前立即停止并写 rejected manifest。--max-buffered-image-bytes（默认
256 MiB）独立限制内存中的原始 Image 字节，超过上限同样在复制前拒绝。manifest 的
summary/limits 记录总接收数、缓存数/字节数和未配对峰值。超时、中断、publisher 数量/类型/QoS
变化、输入缺失或未配对消息都会写 rejected manifest 并返回非零。ROS rgb8 按原像素无损写入
PNG，记录的 SHA-256 针对归档 PNG 字节；width、height、encoding、step、data size 等
字段描述原始 ROS message。

无硬件时完整 ROS 路径由 fake publisher 测试复现：

~~~bash
ctest --test-dir build/auto_aim_tools --output-on-failure \
  -R calibration_dataset_ros_test
~~~

## Manifest 与质量规则

| 区域 | 主要字段 |
| --- | --- |
| 根 | schema_version、manifest_type、status、quality_gate_passed、profile、production_ready |
| 工具/来源 | tool.version、tool.git_commit、created_at_utc、source.mode、input_status、camera_sdk_status、timestamp_source |
| 可追溯项 | 标定板规格、dataset/report ID、设备/镜头/operator、采集日期；IMU/MCU/硬件同步默认 unknown |
| 每条记录 | PNG 相对路径和 SHA-256、原始 stamp 及来源、width/height/encoding/step/data size/frame_id、CameraInfo 存在和三项匹配状态、errors/warnings |
| 安全 | serial_enabled=false、dry_run=true、allow_fire=false、fire_command=0，yaw/pitch 速度和加速度为 0 |

以下任一项拒绝整个数据集：空输入、不可读/空图、零尺寸、非 rgb8、stride/data length
错误、混合分辨率、重复 PNG 哈希、视图不足、标定板缺失或非法、规范时间戳缺失/回退，
以及在要求 CameraInfo 时的缺失、时间戳/尺寸/frame_id 不匹配。相同时间戳按 PR #25
现有契约记录 warning；内容重复仍由哈希规则拒绝。

可独立核对 manifest：

~~~bash
cd /absolute/path/dataset_run_001
sha256sum -c dataset_manifest.sha256
~~~

## 导入标定器与人工复核

通过的数据集生成 calibration_input.yaml，其中 metadata.dataset_manifest_sha256 是
dataset_manifest.yaml 的真实字节哈希。标定器在读取图片前强制复算：

~~~bash
ros2 run auto_aim_ros2 auto_aim_camera_calibrate -- \
  --config /absolute/path/dataset_run_001/calibration_input.yaml \
  --dataset-manifest /absolute/path/dataset_run_001/dataset_manifest.yaml \
  --image-list /absolute/path/dataset_run_001/images.txt \
  --report /absolute/path/dataset_run_001/camera_intrinsic_report.yaml
~~~

哈希不一致、缺少已声明的 manifest 或额外传入未声明 manifest 都会在求解前失败。有关联时，
metadata.dataset_manifest 必须解析到实际传入的同一文件；manifest 中按顺序排列的 accepted
records 是唯一权威图像集。标定器要求 images.txt 与这些相对路径逐项完全一致，并在读取图像前
复算每个归档 PNG 的 SHA-256；替换 image-list、路径或 PNG 内容都会拒绝。旧的
schema v1 配置没有这两个可选 metadata 字段时仍兼容原命令。标定报告继续保持
profile: evidence_only、production_ready: false，并归档已验证的 dataset_manifest_sha256。
候选 K/D 不得写入正式 camera_info.yaml 或 PnP production profile。

人工复核至少检查：每张图中的完整棋盘格和清晰度、视角/距离/画面区域覆盖、设备序列号、
镜头、焦距/光圈固定状态、原始分辨率、标定板实测规格、时间戳来源、重复帧、rejection 和
warning、manifest sidecar，以及标定报告中的逐图重投影误差。任何 unknown 项在得到证据
前不得改写为已确认。

## 真实硬件到位后的模板

本轮不执行以下硬件步骤。完成 MVS 许可证、架构、USB 权限和既有
docs/orin_real_camera_bringup.md 审核后，才可由获批操作者替换占位命令：

~~~bash
source /opt/ros/humble/setup.bash
source /absolute/path/install/setup.bash

# 终端 A：先启动只读录制器
ros2 run auto_aim_tools auto_aim_calibration_dataset_recorder -- \
  --config /approved/path/ros_dataset_config.yaml \
  --output /approved/evidence/calibration_YYYYMMDD_runNN \
  --max-frames 20 --timeout-s 60

# 终端 B：仅在硬件联调审批后执行团队审核过的相机 launch
<approved_camera_launch_command>
~~~

出现任一情况立即停止并保留 rejected evidence：MVS/USB 错误、输入超时、多个 publisher、
非 rgb8、尺寸变化、stride/data length 错误、时间戳为零/回退/不精确配对、frame_id
变化、CameraInfo 缺失/不匹配、重复哈希、镜头或标定板移动、曝光失控、输出路径已存在。
保存位置必须是新的批准证据目录，禁止覆盖旧运行。

功能回退只使用：

~~~bash
git revert <功能提交SHA>
~~~

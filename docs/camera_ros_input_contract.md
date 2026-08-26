# 相机到检测链路 ROS 输入契约

本文只定义 `hik_camera -> ros_input_preflight -> auto_aim_ros2/离线工具` 的输入边界、
启动顺序和排障方法。它不证明 MVS SDK、真实相机、Orin、正式标定、相机到云台外参、
绝对角、串口或实车能力。

## 验证状态

| 能力 | 本任务状态 | 可以得出的结论 |
|---|---|---|
| ROS 输入契约与只读预检 | 由 C++ fixture/fake publisher 覆盖 | 可以判断消息、topic、可见 QoS 和配对是否满足本文契约 |
| MVS SDK、真实海康相机、Orin | 未验证 | 不得写成相机链路或目标平台已通过 |
| 正式内参、外参、绝对角、实车与开火 | 未完成 | 不得把示例 K/D、候选外参或 dry-run 输出用于正式控制 |

## 契约表

| 项目 | 必须满足 | 失败行为 |
|---|---|---|
| 图像 topic/type | `/image_raw`, `sensor_msgs/msg/Image`，恰好一个输入 publisher | `topic.publisher_count` 或 `topic.type` 为 `FAIL` |
| CameraInfo topic/type | `/camera_info`, `sensor_msgs/msg/CameraInfo`，恰好一个输入 publisher | `topic.publisher_count` 或 `topic.type` 为 `FAIL` |
| QoS | 两个 publisher 均为 SensorDataQoS：best effort、volatile、keep last、depth 5 | 可见字段不符时 `FAIL`；DDS graph 不公开 history/depth 时 `WARN`，必须人工复核 |
| 图像布局 | `rgb8`；`width > 0`；`height > 0`；`step == width * 3`；`data.size() == step * height` | 任一项 `FAIL`，该帧不能作为有效检测输入证据 |
| frame_id | Image 与 CameraInfo 均为预期值，默认 `camera_optical_frame`，且同 stamp 配对的两者完全相同 | 单 topic 或配对检查 `FAIL` |
| CameraInfo | 宽高为正并与同 stamp 图像相同；K 全部有限，`fx/fy > 0`、`K[8] != 0`；D 全部有限且长度匹配 distortion model | `camera_info.*` 或 `image_camera.dimensions` 为 `FAIL` |
| distortion model | `plumb_bob`/5、`rational_polynomial`/8 或 `equidistant`/4 | 空值、未知 model 或错误 D 长度为 `FAIL` |
| 时间戳 | Image 与 CameraInfo 使用完全相同的非零 canonical ROS stamp；各 topic 不得回退 | 缺失、非法、回退或旧于 100 ms 仍未配对的消息为 `FAIL` |

100 ms 仅用于避免在观测窗口结束瞬间把仍在 DDS 传输的单条最终尾帧误报为缺失；该情况为
`WARN`，应延长或重复预检。多条未配对消息、中间漏配或超过 100 ms 均为 `FAIL`。这不是允许
相机节点生成 100 ms 的时间戳偏差。`hik_camera` 在一次 `CameraPublisher::publish()` 中发送两条消息，
CameraInfo header 直接复制 Image header，所以节点自身的规则是 stamp 和 frame_id 完全相同。

### 时间戳来源

`/image_raw.header.stamp` 在 SDK 像素转换成功后、发布前由相机节点调用 `this->now()` 取得。
它是运行相机节点的本机 ROS clock 的发布准备时间，不是 SDK 曝光时间、相机硬件时间、IMU
时间或 MCU 时间。仓库没有这些时钟之间的已验证映射。`/camera_info.header.stamp` 复制同一
Image header。

### 参数与实际行为

| 参数 | 默认值 | 行为 |
|---|---|---|
| `camera_serial` | 空 | 只在恰好一台相机时允许隐式选择；多台时必须精确指定 |
| `camera_info_url` | 空 | 发布宽高匹配但 K/D 未标定的 CameraInfo，预检必然失败，检测/PnP 不得启动 |
| `frame_id` | `camera_optical_frame` | 同时用于 Image 和 CameraInfo；空值拒绝启动 |
| `use_sensor_data_qos` | `true` | `/image_raw` 与 `/camera_info` 使用 SensorDataQoS |

仓库内 `src/ros2-hik-camera/config/camera_info.yaml` 仅是未验证格式示例，launch 不再默认加载，
节点也显式拒绝其 `package://hik_camera/config/camera_info.yaml` URI。正式文件必须从外部受控
路径显式传入。图像分辨率来自每个成功转换的 SDK frame，不是 launch 中的猜测值；若正式
CameraInfo 宽高不匹配，整帧拒绝发布并留下明确错误。

## 无硬件构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  auto_aim_interfaces hik_camera auto_aim_tools auto_aim_ros2
source install/setup.bash
colcon test --packages-select hik_camera auto_aim_tools auto_aim_ros2 \
  --event-handlers console_direct+
colcon test-result --verbose
git diff --check
```

没有 MVS 动态库时，`hik_camera` 真实可执行程序会被跳过；相机节点仍执行 compile-check，
`camera_safety_test` 和 `auto_aim_tools` 测试不依赖 SDK。该结果必须写作“SDK 路径未验证”，
不能写成真实相机通过。

回归覆盖有效 `rgb8 + CameraInfo`，缺 CameraInfo，无效 K/D，宽高不匹配，空帧、零宽高、
stride/data/encoding 错误，时间戳缺失、回退和不匹配，frame_id、topic、QoS、参数错误，
SIGINT/SIGTERM/重复停止/强制退出后的进程回收，以及预检节点没有 publisher/service/client。
检测侧既有 `ros_image_adapter_test` 继续覆盖有效 rgb8 转 BGR、空帧、短 stride/data、未知编码
和非法时间戳；本任务不修改检测、PnP、Tracker、TargetSelector、Aimer 或 RobotCtrl 实现。

## 可复现启动顺序

所有终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /path/to/worktree/install/setup.bash
```

### 1. 只启动相机输入

仅在另行批准硬件联调后使用；本任务没有执行此命令：

```bash
ros2 launch hik_camera hik_camera.launch.py \
  camera_serial:=EXACT_SERIAL \
  camera_info_url:=file:///absolute/path/to/verified_camera_info.yaml \
  frame_id:=camera_optical_frame \
  use_sensor_data_qos:=true
```

### 2. 运行只读预检

不要同时启动 `auto_aim_node`、串口桥或控制节点：

```bash
ros2 run auto_aim_tools ros_input_preflight \
  --duration 10 \
  --timeout 1.0 \
  --expected-frame-id camera_optical_frame \
  --vehicle-profile unselected \
  --format json \
  --output /tmp/ros_input_preflight.json
echo $?
```

退出码 `2` 表示至少一项 `FAIL`，不得继续启动检测。退出码 `0` 仍可能包含 `WARN`；尤其是
Fast DDS 未公开 history/depth 时必须执行下面的人工检查：

```bash
ros2 topic list | grep -E '^/(image_raw|camera_info)$'
ros2 topic info /image_raw -v
ros2 topic info /camera_info -v
ros2 topic hz /image_raw
ros2 topic echo --once /image_raw
ros2 topic echo --once /camera_info
ros2 node info /ros_input_preflight
```

`ros_input_preflight` 只能出现 `/image_raw`、`/camera_info`、`/Vision_data` 三个订阅，不能出现
任何 publisher；尤其不能创建 `/Robot_ctrl_data` publisher 或发送控制消息。

### 3. 契约通过后再启动检测或离线链路

先停止预检并保存报告。ROS null backend 的安全启动参数必须显式保留：

```bash
ros2 run auto_aim_ros2 auto_aim_node --ros-args \
  -p backend:=null \
  -p require_camera_info:=true \
  -p dry_run:=true \
  -p serial_enabled:=false \
  -p allow_fire:=false
```

`auto_aim_node` 会按既有协议发布安全 RobotCtrl，这与“预检工具零 publisher”是两个不同边界。
本任务不运行或修改该控制链路。`offline_reference`、`auto_aim_detector_smoke` 和
`auto_aim_offline` 只有在显式提供已核验模型、录像、model profile 和 PnP 配置时才可运行；
其命令与 test-only 限制见 `docs/auto_aim_ros2_design.md`，不得用占位文件伪造通过。

## 停止与排障

正常停止使用一次 Ctrl-C。预检收到 SIGINT 返回 `130`，SIGTERM 返回 `143`，两者都会先写出
报告；重复停止不会留下子进程。强制退出只用于测试回收边界，不是现场常规停止方法。

```bash
ros2 node list
ps -ef | grep -E 'hik_camera|ros_input_preflight|auto_aim_node' | grep -v grep
```

| 诊断 | 常见原因 | 处理 |
|---|---|---|
| required topic 无 publisher | 节点未启动、namespace/remap 错、异常退出 | 核对根 topic 和节点日志，不启动检测 |
| publisher count 大于 1 | fake 与真实源并存、重复节点 | 停止重复源，避免混合不同帧 |
| QoS `FAIL` | reliable/transient local 或明确的 history/depth 错误 | 两个相机 topic 都改回 SensorDataQoS |
| QoS `WARN` | DDS graph 不公开 history/depth | 用 `ros2 topic info -v` 确认 keep last/depth 5 |
| encoding/step/data `FAIL` | 非 rgb8、行跨度或缓冲区长度错误 | 修复发布端；异常帧不得进入检测证据 |
| CameraInfo `FAIL` | URL 空、候选/旧标定、NaN/Inf、D model/长度错误 | 停止 PnP，换用有来源记录的正式标定 |
| pairing/frame_id/dimensions `FAIL` | 分辨率改变、两个 publisher、header 未复制或消息丢失 | 核对同一 CameraPublisher 与相机日志 |
| timestamp rollback/unset | ROS clock 配置错误或 publisher 填充错误 | 停止链路；不得宣称硬件同步 |
| MVS library missing | SDK 未安装、架构或动态库路径错误 | 记录为未验证，不得用 compile-check 代替硬件结果 |

任何阶段都保持 `serial_enabled=false`、`dry_run=true`、`allow_fire=false`、`fire_command=0`。

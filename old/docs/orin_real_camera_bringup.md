# Orin 与真实相机联调准备清单

本文只描述 `game_26_dev` 在 Orin 和海康 USB3 相机联调前需要准备、检查和留证的内容。
截至本文编写时，没有在 Orin 上编译，也没有连接真实相机、串口或机器人；文中的命令是
上线前的检查流程，不代表硬件验证已经通过。

ROS Image/CameraInfo 的精确字段、QoS、配对规则和预检启动顺序以
[`camera_ros_input_contract.md`](camera_ros_input_contract.md) 为准。

## 1. 当前审计结论

| 组件 | 当前仓库依赖 | 当前证据/状态 | 上线前动作 |
|---|---|---|---|
| `auto_aim_interfaces` | `std_msgs`、ROS 2 interface generation | 消息包可由 `colcon` 构建 | 与视觉程序、串口桥使用同一工作区并先 source |
| `serical_device_ros2` | `rclcpp`、`rclcpp_components`、`message_filters`、`auto_aim_interfaces`；Linux 串口 API | dry-run/协议测试不需要设备 | MCU watchdog 约 500 ms 且车型可能不同；只在确认 golden frame、波特率和精确 watchdog 后再启用设备 |
| `auto_aim_ros2` | `rclcpp`、`sensor_msgs`、`auto_aim_interfaces`、`yaml-cpp`；CMake 查找 OpenCV；OpenVINO 为可选构建依赖 | 自定义 `ros_image_adapter`，当前不依赖 `cv_bridge`；`calib3d` 在 PnP 中使用 | 安装 OpenCV development、yaml-cpp 和 OpenVINO Runtime；验证模型签名 |
| `hik_camera` | `rclcpp`、`rclcpp_components`、`sensor_msgs`、`image_transport`、`camera_info_manager`、`image_transport_plugins`；海康 MVS SDK | 仓库当前只有 `hikSDK/include`，没有 `hikSDK/lib/amd64` 或 `hikSDK/lib/arm64`；WSL 当前未选择该包构建 | 获取与目标架构匹配且可再分发的 SDK 动态库，完成 USB 权限和 `ldd` 检查 |

`auto_aim_ros2` 的 CMake 明确查找以下 OpenCV 模块：`calib3d`、`core`、`dnn`、`imgcodecs`、
`imgproc`、`videoio`。OpenVINO 查找失败时仍允许通信/dry-run 包编译，但检测器在运行时会报告
不可用；要使用 `offline_reference`，必须显式提供可用的 OpenVINO Runtime 和模型。

当前相机节点发布 `image_raw` 和配套 `camera_info`，默认编码为 `rgb8`，而自瞄节点的图像适配层
会将 `rgb8` 转为拥有所有权的 BGR `cv::Mat`。因此当前方案不需要 `cv_bridge`。若未来改用
`cv_bridge::toCvShare` 或 `toCvCopy`，必须同步在 `auto_aim_ros2/package.xml`、CMake 和 Orin
安装清单中增加 `cv_bridge`，并重新验证 encoding、step 和 QoS；不能只安装但不说明边界。

相机节点已经使用实际 rgb8 size 执行 `resize()`，检查转换输出长度，声明白平衡参数，并对
枚举、创建、打开、转换、释放和停止路径保留 SDK 状态诊断。没有显式序列号时只允许恰好
一台相机；多台、找不到指定序列号或序列号重复都会拒绝启动。这些是 compile-check 与纯 C++
回归覆盖的代码边界，不是 MVS SDK 或真实 USB 相机已验证的证据。

## 2. WSL 基线检查（只读）

执行目录：`/home/ubuntu22/vision-study/game_26_dev`。以下命令不连接硬件、不发布控制命令：

```bash
cd /home/ubuntu22/vision-study/game_26_dev
source /opt/ros/humble/setup.bash

colcon list --names-only
pkg-config --modversion opencv4
test -f /usr/include/opencv4/opencv2/calib3d.hpp
test -f /opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake
ros2 pkg prefix cv_bridge || true       # 当前自定义适配器不要求它
find src/ros2-hik-camera/hikSDK -type f -name 'lib*.so*' -print
```

本次审计观察到：WSL 为 `x86_64`，OpenCV `pkg-config` 版本为 4.1.0，`calib3d.hpp` 和
OpenVINO CMake 配置存在；仓库内海康 SDK 动态库为空。`camera_info_manager` 和
`image_transport_plugins` 需要在目标机通过 ROS 安装包补齐，不能把 WSL 中其他包的传递依赖
当作 `hik_camera` 的直接安装证明。

## 3. Orin 软件安装与编译顺序

以下以 Ubuntu 22.04、ROS 2 Humble、目标架构 `aarch64` 为前提。版本号、APT 源和 OpenVINO
安装位置必须记录在联调报告中；不要从旧 Windows 副本复制二进制文件。

1. 安装 ROS 基础依赖：

   ```bash
   sudo apt update
   sudo apt install \
     ros-humble-rclcpp ros-humble-rclcpp-components \
     ros-humble-ament-cmake-auto ros-humble-message-filters \
     ros-humble-rcl-interfaces ros-humble-rosidl-default-generators \
     ros-humble-sensor-msgs ros-humble-image-transport \
     ros-humble-image-transport-plugins \
     ros-humble-camera-info-manager ros-humble-cv-bridge \
     libopencv-dev libyaml-cpp-dev
   ```

   在目标机也应优先用工作区依赖清单复核一次（`rosdep` 未初始化时先按团队
   的系统管理流程初始化和更新索引）：

   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
   ```

   上面的显式列表补充了 `serical_device_ros2` 直接使用的
   `message_filters`、相机参数回调使用的 `rcl_interfaces`，以及接口包和
   `ament_cmake_auto` 的构建依赖；它不是对 SDK/OpenVINO 动态库的替代证明。

   `cv_bridge` 是可选的兼容工具；当前代码使用自定义适配器，但安装它有助于用标准工具
   检查图像 encoding。它不会自动改变当前节点的数据流。

2. 安装与 Orin/aarch64 匹配的 OpenVINO Runtime。确认 `setupvars.sh` 或等效环境脚本、
   CPU/plugin 动态库和 CMake 配置都来自同一个版本：

   ```bash
   source /opt/intel/openvino_*/setupvars.sh
   test -f "$OpenVINO_DIR/OpenVINOConfig.cmake" || true
   ldconfig -p | grep -E 'openvino|openvino_intel_cpu_plugin' || true
   ```

   若使用非默认路径，构建时显式传入 `-DOpenVINO_DIR=/path/to/runtime/cmake`，不要依靠
   个人 shell 的隐式环境变量。

3. 在海康 MVS SDK 中选择 `aarch64/arm64` 版本，并将头文件和动态库按当前 CMake 约定放置：

   ```text
   src/ros2-hik-camera/hikSDK/include/
   src/ros2-hik-camera/hikSDK/lib/arm64/
     libFormatConversion.so
     libMediaProcess.so
     libMvCameraControl.so
     libMVRender.so
     libMvUsb3vTL.so
   ```

   文件名以 SDK 实际发布为准，但必须能够满足 CMake 中的
   `FormatConversion`、`MediaProcess`、`MvCameraControl`、`MVRender`、`MvUsb3vTL` 五个链接名。
   SDK 的许可证和版本号应写入交付记录；不能提交未经许可的 SDK 二进制。

4. 编译前先检查架构和 SDK 库，不需要连接相机：

   ```bash
   uname -m
   find src/ros2-hik-camera/hikSDK/lib/arm64 -maxdepth 1 -type f -name 'lib*.so*' -print
   file src/ros2-hik-camera/hikSDK/lib/arm64/libMvCameraControl.so
   ```

   期望 `uname -m` 为 `aarch64`，SDK 库也显示 `ARM aarch64`。若目录缺失或 `file` 显示
   x86-64，应停止，不要让 CMake 静默链接错误架构。

5. 在仓库目录构建，先不编译/运行真实串口：

   ```bash
   cd /path/to/game_26_dev
   source /opt/ros/humble/setup.bash
   source /opt/intel/openvino_*/setupvars.sh  # 若采用 OpenVINO 安装脚本

   colcon build --symlink-install \
     --packages-select auto_aim_interfaces serical_device_ros2 auto_aim_ros2 hik_camera \
     --cmake-args -DOpenVINO_DIR="$OpenVINO_DIR"
   source install/setup.bash
   ldd install/hik_camera/lib/libhik_camera.so
   ldd install/auto_aim_ros2/lib/libauto_aim_ros2.so
   ```

   `ldd` 不得出现 `not found`。当前 WSL 没有海康 SDK 库，所以不能用 WSL 的构建结果替代
   这一步；也不要将 `build/`、`install/` 里的 x86_64 二进制拷贝到 Orin。

## 4. 海康相机与 ROS 话题检查

### 4.1 启动前检查

相机节点 `hik_camera` 在只发现一台相机时允许隐式选择；接入多台时必须用 `camera_serial`
精确指定且序列号必须唯一。仍应在日志中记录序列号/型号。启动前检查：

```bash
lsusb
ls -l /dev/bus/usb/*/*
ros2 pkg prefix hik_camera
```

MVS/USB 访问失败、设备数为 0、SDK 版本不匹配时应停止，不要反复重启节点掩盖问题。若团队
使用 udev 规则，应记录规则文件、组权限和生效后的 `udevadm info`，并用普通用户验证访问；
不建议把整套节点作为 root 运行。

### 4.2 相机内参和参数文件

`src/ros2-hik-camera/config/camera_info.yaml` 是零尺寸、零 K 的不可用格式示例；节点在硬件
初始化前按规范路径和文件身份拒绝它的 URL/文件别名，不能作为 production 标定证据。
正式文件应生成到外部受控路径并记录：

- 相机序列号、镜头和固件；
- 原始图像宽高、像素格式、裁剪/缩放/letterbox；
- `K`、`D`、畸变模型和重投影误差；
- 标定板类型、尺寸、采集日期、工具和版本。

当前节点默认 `camera_name: narrow_stereo`、`camera_info_url: ""`、
`frame_id: camera_optical_frame`、`use_sensor_data_qos: true`，默认曝光 3000、增益 15。
URL 为空时发布明确未标定的 K/D，预检会失败。仓库内候选 YAML 的 package URI 被显式拒绝；
上车前必须通过 launch 参数提供受控的实测文件，并确认相机发布分辨率与 PnP YAML 完全一致。
不允许用 `CameraInfo.P` 冒充原始图像的 `K`，也不允许自动猜测缩放比例。

相机节点固定发布根 topic `/image_raw` 和 `/camera_info`，与当前 `auto_aim_node` 的绝对订阅
一致。不要用 remap 或第二个 fake publisher 绕过预检；必须在 `ros2 topic list` 和
`ros2 topic info -v` 中记录最终 graph。

### 4.3 话题、编码和 QoS

执行目录：已 source ROS 和工作区环境的 Orin 终端。

```bash
ros2 launch hik_camera hik_camera.launch.py \
  frame_id:=camera_optical_frame \
  use_sensor_data_qos:=true \
  camera_info_url:=file:///absolute/path/to/verified_camera_info.yaml
```

另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /path/to/game_26_dev/install/setup.bash

ros2 topic list | grep -E '^/(image_raw|camera_info)$'
ros2 topic info /image_raw -v
ros2 topic info /camera_info -v
ros2 topic hz /image_raw
ros2 topic echo --once /image_raw/header
ros2 topic echo --once /camera_info
```

必须人工记录：

- `/image_raw` 是否为 `sensor_msgs/msg/Image`、宽高、`encoding`、`step` 和频率；
- `/camera_info` 是否为同一帧时间戳、同一宽高，K/D 是否有限且 `fx/fy > 0`；
- QoS 是否与 `auto_aim_node` 的 `SensorDataQoS` 兼容；
- `header.frame_id` 是否稳定且与标定记录一致。

当前 `hik_camera` 发布 `rgb8`；自瞄节点会转换成 BGR 后再交给 OpenCV/OpenVINO。若现场改为
`bgr8`、Mono 或压缩传输，必须重新跑图像适配测试和模型颜色顺序检查。

## 5. 安全 dry-run 启动顺序

真实相机第一次联调只允许图像和日志链路，必须显式保持：

```text
backend=null 或 backend=offline_reference
dry_run=true
serial_enabled=false
allow_fire=false
fire_command=0
```

在确认图像、CameraInfo、模型签名、PnP 配置来源和 CSV 后，才可在同一台机器上接入更多
ROS 节点。串口设备权限、波特率和 RobotCtrl golden frame 未经电控确认前，不得启动会打开
`/dev/tty*` 的配置；`fire_command=1/2` 不属于本阶段验证范围。

检测节点启动前先单独运行只读预检并保存报告：

```bash
ros2 run auto_aim_tools ros_input_preflight \
  --duration 10 \
  --timeout 1.0 \
  --expected-frame-id camera_optical_frame \
  --format json \
  --output /tmp/ros_input_preflight.json
```

存在任何 `FAIL` 都必须停止，不得继续启动检测。预检本身没有 publisher，不能创建
`/Robot_ctrl_data`。只有契约通过并人工复核 QoS history/depth 后，才进入下面的 null backend。

推荐先用无硬件的 Null backend 验证发布器：

```bash
ros2 run auto_aim_ros2 auto_aim_node --ros-args \
  -p backend:=null \
  -p dry_run:=true \
  -p serial_enabled:=false \
  -p allow_fire:=false
```

`offline_reference` 只有在显式提供已核验的模型、PnP 配置且允许 test-only 时才能使用。没有
真实 camera→gimbal 外参或绝对零点时，只能记录 camera pose/relative angle，不能宣称具备正式
RobotCtrl 绝对角控制能力。

## 6. 联调记录和停止条件

每次 Orin/相机尝试至少保存：

- commit、Orin 型号、JetPack/Ubuntu/ROS/OpenVINO/MVS 版本；
- `uname -m`、SDK `file`/`ldd` 输出；
- 相机序列号、固件、曝光/增益、分辨率和帧率；
- `/image_raw`、`/camera_info` 的 QoS、编码、时间戳和 `frame_id`；
- 使用的 calibration profile、model version、PnP 重投影误差；
- 使用的 detector model profile（若有）、model artifact hash 和版本；
- dry-run 日志、CSV、异常帧和明确的 `fire_command=0` 证据。

出现以下任一情况立即停止并回到离线验证：SDK 动态库 `not found` 或架构不符、CameraInfo
缺失/尺寸不符、图像 encoding/step 不受支持、模型签名不符、标定来源不明、ROS topic QoS
不兼容、输入超时保护未验证、串口/开火配置无法证明为关闭。

## 7. 日志与故障排查最小流程

每次尝试先建立独立的记录目录；日志、CSV 和异常帧只写入该目录或 `/tmp`，不要把
设备序列号以外的隐私信息、口令、token、SDK 二进制或未审查模型加入 Git：

```bash
run_id=$(date +%Y%m%d_%H%M%S)
record_dir=/tmp/game26-bringup/${run_id}
mkdir -p "${record_dir}"

ros2 launch hik_camera hik_camera.launch.py \
  frame_id:=camera_optical_frame \
  use_sensor_data_qos:=true \
  camera_info_url:=file:///absolute/path/to/verified_camera_info.yaml \
  2>&1 | tee "${record_dir}/hik_camera.log"
```

在另一终端保存节点、话题、参数和内核 USB 信息：

```bash
ros2 node info /hik_camera | tee "${record_dir}/node_info.txt"
ros2 topic info /image_raw -v | tee "${record_dir}/image_qos.txt"
ros2 topic info /camera_info -v | tee "${record_dir}/camera_info_qos.txt"
ros2 topic hz /image_raw | tee "${record_dir}/image_hz.txt"
ros2 topic echo --once /camera_info | tee "${record_dir}/camera_info_once.txt"
ros2 param dump /hik_camera | tee "${record_dir}/params.yaml"
uname -a | tee "${record_dir}/uname.txt"
dmesg --ctime | tail -100 | tee "${record_dir}/dmesg_tail.txt"
```

按第一条可复现证据分类，不要用反复重启掩盖故障：

| 现象 | 首要检查 | 停止条件/处理 |
|---|---|---|
| 设备数为 0、创建/打开失败 | MVS 版本、USB 线、电源、udev 权限、SDK 返回码 | 记录返回码、型号和权限；未恢复前不启动检测器 |
| 转换失败或图像数据为空 | `ConvertPixelType` 返回码、目标缓冲区 size/step、源像素格式 | 回到相机代码修复 `reserve`/`resize` 问题；不得发布空帧 |
| 话题没有数据或 QoS 不匹配 | 最终 topic 名、`ros2 topic info -v`、编码/宽高/时间戳 | 保持 `serial_enabled=false`，修正 launch/remap/QoS 后再试 |
| CameraInfo 缺失、尺寸或 K/D 不符 | URL、序列号、分辨率和标定版本 | 停止 PnP；禁止把候选 YAML 当 production |
| OpenVINO/模型加载失败或 `ldd not found` | `OpenVINO_DIR`、模型 profile、`file`/`ldd` 架构 | 停止真实链路，回到离线 smoke；不得复制 WSL 二进制到 Orin |
| 图像/输入超时 | `ros2 topic hz`、节点时间戳和 timeout 日志 | 只允许安全保持输出；本机视觉侧 100 ms timeout 不替代约 500 ms、车型相关的 MCU watchdog；未验证 watchdog 前不得接串口 |

每条记录都要带 commit、软件/SDK 版本、命令、开始/结束时间和停止原因；只要
`fire_command`、串口开关或标定来源无法证明为安全状态，就回到 `backend=null`、
`dry_run=true`、`serial_enabled=false`、`allow_fire=false` 的启动方式。

# ROS 2 自瞄输入预检工具

`auto_aim_tools/ros_input_preflight` 是 C++17 与 `rclcpp` 实现的独立只读诊断工具，用于接入 Orin、真实相机或实车前检查输入。它只订阅：

相机到检测链路的完整字段契约、启动顺序和故障表见
[`camera_ros_input_contract.md`](camera_ros_input_contract.md)。

- `/image_raw`（`sensor_msgs/msg/Image`）
- `/camera_info`（`sensor_msgs/msg/CameraInfo`）
- `/Vision_data`（`auto_aim_interfaces/msg/Vision`）

它不发布 `/Robot_ctrl_data` 或任何其他 topic，不设置 ROS 参数，不打开相机、海康 SDK 或串口，也不对输入消息做修改。节点采用最小 `rclcpp` 接口构造，不创建 rosout、参数事件、参数服务或客户端；ROS graph 中只有上述三个订阅。它没有 `serial_enabled`、`dry_run`、`allow_fire` 或 `fire_command` 开关；仓库既有安全默认值仍保持 `serial_enabled=false`、`dry_run=true`、`allow_fire=false`、`fire_command=0`。

## 构建与测试

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select auto_aim_interfaces auto_aim_tools

source install/setup.bash

colcon test \
  --packages-select auto_aim_tools \
  --event-handlers console_direct+

colcon test-result --verbose
ament_uncrustify src/auto_aim_tools/include src/auto_aim_tools/src src/auto_aim_tools/test
ament_xmllint src/auto_aim_tools/package.xml
git diff --check
```

`preflight_analyzer_test.cpp` 用普通 C++ sample 覆盖消息字段、配对和 graph finding；
`fake_ros_publishers_test.cpp` 使用 `rclcpp` fake publisher 覆盖有效 rgb8、缺 CameraInfo、
K/D、宽高、frame_id、encoding、stride、data、时间戳、topic 和 QoS 错误，并检查节点图只有
三个输入订阅且没有 publisher/service/client。`process_contract_test.cpp` 启动真实预检进程，
验证参数错误、正常/失败退出、SIGINT、SIGTERM、重复停止和强制退出后的回收。测试不连接 SDK
或硬件；准确 case 数和结果以本提交的 `colcon test-result --verbose` 为准。

## Orin 与实车前执行

先只启动待检查的输入源。不要启动 `auto_aim_node`、串口桥或控制节点。选择已确认的车型 profile 后执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run auto_aim_tools ros_input_preflight \
  --duration 10 \
  --timeout 1.0 \
  --expected-frame-id camera_optical_frame \
  --vehicle-profile new_turtle \
  --format text
```

可选 profile 及其仅用于诊断的 pitch 边界为：

| profile | pitch 范围 |
| --- | --- |
| `new_turtle` | `[-20 degree, 19 degree]` |
| `dog_leg` | `[-10 degree, 31 degree]` |
| `unselected`（默认） | 无法判定，报告 `WARN` |

如需机器处理，可输出 JSON：

```bash
ros2 run auto_aim_tools ros_input_preflight \
  --duration 10 \
  --timeout 1.0 \
  --expected-frame-id camera_optical_frame \
  --vehicle-profile dog_leg \
  --format json \
  --output /tmp/ros_input_preflight.json
```

机器读取时必须使用 `--output`，不要把 `ros2 run` 的 stdout 直接重定向为 JSON。预检失败会保留退出码 `2`，而 `ros2run` 会在 stdout 追加失败提示；`--output` 将报告直接写入指定文件，因此该文件仍是单个合法 JSON 文档。写文件失败同样返回 `2`，不会留下被当作成功报告的 stdout JSON。

默认不比较 Image 与 Vision 的 Header 时间戳，并报告“时间基准未确认”。只有硬件/ROS 负责人已经明确二者使用同一时钟域时，才允许执行：

```bash
ros2 run auto_aim_tools ros_input_preflight \
  --duration 10 \
  --timeout 1.0 \
  --expected-frame-id camera_optical_frame \
  --vehicle-profile new_turtle \
  --assume-shared-clock-domain \
  --sync-tolerance-ms 50
```

该选项只计算最新两条消息的时间戳差，不宣称消息已同步，也不做插值或方向推断。

## 检查与结果

报告中的状态含义：

- `PASS`：本次观测满足该项可验证的格式要求。
- `WARN`：信息不足、统计样本不足或需人工确认；不应被写成已验证。
- `FAIL`：缺/重复/wrong-type topic、可确认的 QoS 错误、格式/有限值/范围错误、Image/CameraInfo 配对错误、时间戳缺失/回退或接收超时。

进程在存在 `FAIL` 时返回退出码 `2`，只有 `PASS/WARN` 时返回 `0`，收到 Ctrl-C 时返回 `130`。瞬时坏帧不会被后续正常帧覆盖；异常回调被记录为 `FAIL`，工具继续收集并输出报告。

文本报告节选：

```text
ROS 2 INPUT PREFLIGHT: WARN
[PASS] /image_raw image.encoding: Image encoding matches the camera contract | actual=rgb8, expected=rgb8
[PASS] /camera_info camera_info.K: K has 9 finite entries with positive fx/fy and non-zero K[8] | length=9
[PASS] cross_topic image_camera.timestamp_pairing: Every Image has a CameraInfo with the exact same non-zero Header timestamp
[PASS] /Vision_data vision.yaw_range: yaw is within inclusive [-180 degree, 180 degree] | yaw=12.5 degree
[WARN] /Vision_data vision.acceleration_finite: Installed Vision interface has no acceleration field; degree/s^2 check is unavailable
[WARN] cross_topic image_vision.clock_domain: Shared clock domain was not explicitly declared: 时间基准未确认; timestamps were not compared
summary: PASS=18, WARN=2, FAIL=0
```

Vision 数值只按接口外部单位报告和检查：`yaw/pitch/roll` 为 degree，`yaw_vel/pitch_vel` 为 degree/s，存在时 `yaw_acc/pitch_acc` 为 degree/s²。工具不会换算后覆盖消息。yaw 必须在闭区间 `[-180 degree, 180 degree]`；pitch 只有显式 profile 才判定。

quaternion 只检查恰为 4 项、接口声明顺序为 `wxyz` 且元素有限。它可记录为“IMU 相对于上电原点”，但不归一化、不旋转、不做坐标变换、不推断参考系方向，也不将其用于 Tracker/Aimer。

## 已知限制与未验证项

- 当前 `origin/main` 的 `auto_aim_interfaces/msg/Vision.msg` 没有 `yaw_acc`、`pitch_acc` 字段。工具兼容未来字段；当前运行会明确 `WARN`“无法检查”，不会修改接口补字段。
- 频率使用本机单调时钟和订阅到达时间统计，不等同于传感器曝光频率、端到端延迟或 DDS 无丢包证明。
- 相机到检测契约只接受 `rgb8`；其他 encoding 直接 `FAIL`。同时要求 `step == width * 3` 和 `len(data) == step * height`。工具不解码像素，也不启动相机验证画质。
- `CameraInfo` 检查尺寸、K、D、有限值及常见 distortion model 的长度，不验证标定精度、外参或重投影误差。
- 每条已收到的 Image 都必须有精确同 stamp 的 CameraInfo；任一未配对 Image 直接 `FAIL`。若所有已收到 Image 都精确配对、配对字段均通过，而只多收到 CameraInfo，则明确报告 surplus/stale 数量并降为 `WARN`。这可能表示 BEST_EFFORT 下 image-side DDS delivery loss，不是源端错配结论，也不是端到端同步、无丢包或无损投递证明。时间戳非法/回退和任何实际字段错配仍 `FAIL`。跨 Image/Vision 比较仍必须显式声明共同时间域，比较结果也不是同步证明。
- Humble/Fast DDS 可能在 graph API 中把 history/depth 返回为 unknown/0。此时 best-effort/volatile 可自动检查，但 keep-last/depth 5 报告 `WARN`，必须用 `ros2 topic info -v` 人工复核。
- quaternion 的真实方向、参考系和传感器时序未在本工具中验证。
- fake publisher 是离线 ROS 验收证据，不替代 Orin、真实相机、下位机或实车记录。

## 回退方法

运行时回退只需停止 `ros_input_preflight`；它没有发布、参数或设备状态需要恢复。代码回退使用交付的功能提交执行 `git revert <功能提交 SHA>`。不要使用 `reset --hard`、`checkout --` 或 `git clean`。

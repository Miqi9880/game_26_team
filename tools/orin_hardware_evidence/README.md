# Orin 环境只读预检工具

`orin_environment_preflight` 是独立 C++17 工具，只读取操作系统、架构、Orin device-tree
型号标识、环境变量和依赖文件元数据。它不依赖 ROS 运行时编译，不发布 ROS topic，不打开相机
或串口，不加载海康 MVS SDK，也不修改系统配置。

默认不检查任何 `/dev` 路径。只有显式传入 `--camera-device` 或 `--serial-device` 时，工具才检查
路径是否存在及当前进程的读写权限；该检查使用文件系统权限查询，仍不会打开或读写设备。

## 构建与运行

为避免在仓库中产生构建产物，建议把构建目录放在 `/tmp`：

```bash
cmake -S tools/orin_hardware_evidence -B /tmp/game26-orin-preflight-build \
  -DBUILD_TESTING=ON
cmake --build /tmp/game26-orin-preflight-build
ctest --test-dir /tmp/game26-orin-preflight-build --output-on-failure

/tmp/game26-orin-preflight-build/orin_environment_preflight \
  --repo-root "$PWD"
```

只有需要核对已有设备节点的权限元数据时才显式增加：

```bash
/tmp/game26-orin-preflight-build/orin_environment_preflight \
  --repo-root "$PWD" \
  --camera-device /dev/video0 \
  --serial-device /dev/robomaster
```

不要为运行本工具连接相机、串口、Orin 或机器人。不要为得到“好看”的结果创建假设备文件、
伪造环境变量或复制错误架构的 SDK 库。

## 状态和退出码

| 退出码 | 状态 | 含义 |
|---:|---|---|
| `0` | `ENVIRONMENT_PREFLIGHT_COMPLETE_NOT_HARDWARE_VALIDATED` | 仅发现目标环境和依赖元数据；不代表硬件通过 |
| `2` | `非目标环境 (NON_TARGET_ENVIRONMENT)` | WSL、非 Linux、非 aarch64/arm64 或没有 Orin 型号证据 |
| `3` | `缺依赖 (MISSING_DEPENDENCIES)` | 目标环境缺少 ROS 2、OpenCV、OpenVINO 或 MVS SDK 元数据 |
| `4` | `非目标环境/缺依赖` | 两类问题同时存在 |
| `64` | 参数错误 | 命令行参数不完整或未知 |

即使退出码为 `0`，报告也固定输出 `hardware_validation=NOT_RUN`。设备权限字段只是元数据，不能
证明相机可采图、CDC USB 通信正常、固件匹配、CRC 正确、云台可控或发射安全。

## 检查范围

- ROS 2：`ros2` 是否位于 `PATH`，`ROS_DISTRO` 是否已设置；
- OpenCV：常见系统路径是否存在 development header；
- OpenVINO：`OpenVINO_DIR/OpenVINOConfig.cmake` 或 `/opt/intel/openvino*/runtime/cmake`；
- MVS：仓库 SDK header 及 `hikSDK/lib/arm64` 下当前 CMake 所需的五个动态库；
- Orin：Linux、非 WSL、aarch64/arm64 和 `/proc/device-tree/model` 中的 Orin 标识。

这些是存在性检查，不执行 `ldd`、不校验动态库 ABI，也不调用 SDK。版本、架构、ABI、许可证和
真实设备行为必须在实机记录中另行留证。运行时回退只需停止该进程；它没有需要恢复的设备或
系统状态。代码回退使用 Draft PR 的 revert，不改协议或核心节点。

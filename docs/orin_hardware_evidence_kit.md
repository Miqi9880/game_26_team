# Orin/硬件联调证据包使用说明

本证据包用于在真实 Orin、相机和电控联调前固定环境检查与取证格式。仓库内只提供只读工具和
空白模板，不包含真实设备证据，也不能替代电控提供的原始帧、CRC 源码、固件和 ABI 说明。

## 安全边界

本任务没有打开相机、串口或海康 SDK，没有连接 Orin、机器人或云台，没有控制运动或发射，
也没有修改协议、CRC、parser、帧结构、帧长度、串口桥、ROS msg、RobotCtrl、检测、PnP、
Tracker、Target、Aimer、模型或正式标定。所有示例固定为：

```text
serial_enabled=false
dry_run=true
allow_fire=false
fire_command=0
```

通信链路记录为 **CDC USB**。当前没有经电控与实机证据确认的波特率；代码中出现的 `115200`
只能记为“待核验的实现值”，不能写入“已确认波特率”字段，也不能据此宣称链路参数已确认。

## 文件与流程

1. 按 [`tools/orin_hardware_evidence/README.md`](../tools/orin_hardware_evidence/README.md)
   构建并运行只读预检。WSL、Windows、x86_64 或无 Orin 型号证据必须保留“非目标环境”；
   ROS 2、OpenCV、OpenVINO 或 MVS 不完整必须保留“缺依赖”。不得手工改写成通过。
2. 每次实际尝试复制 [`orin_hardware_experiment_record.md`](templates/orin_hardware_experiment_record.md)
   到受控的外部证据目录填写。不要把设备序列号、个人路径、token、未经审查的标定、模型、
   SDK 二进制或敏感日志提交到仓库。
3. 联调串口前，把 [`electrical_golden_frame_request.md`](templates/electrical_golden_frame_request.md)
   交给电控填写。必须取得来自真实固件/链路的原始十六进制帧和独立 CRC 校验，模板本身不算证据。
4. 将记录 ID、访问受控证据的方式、校验 hash 和评审结论关联到 Issue/PR；不在公开文本中粘贴
   凭据、个人绝对路径或敏感设备信息。

## 当前协议边界（仅记录）

以下内容来自本任务给定边界和当前主线，仅作采集验收基线，不在本证据包中修改：

| 数据 | 方向 | 命令号 | payload | 当前完整帧边界 |
|---|---|---:|---:|---|
| `VisionData` | 下位机到上位机 | `VISION_ID=0x0105` | 47 bytes | 58 bytes；CRC16 后有 `0D 0A` |
| `RobotCtrlData` | 上位机到下位机 | `CHASSIS_CTRL_CMD_ID=0x0102` | 26 bytes | 35 bytes；CRC16 后不追加 `0D 0A` |

`RobotCtrlData` 的 35-byte 边界已有合并的
[PR #13](https://github.com/Miqi9880/game_26_team/pull/13) 记录。它不自动证明现场固件、CRC、
端序或 ABI 与主线一致。`header/data_length/seq` 的原始字节、命令号字节顺序、payload 字节、
CRC 覆盖范围与结果仍必须从 golden frame 和电控证据逐项填写，禁止按模板占位符猜测。

## 判定原则

- 预检退出码 `0` 只表示目标环境与依赖元数据已发现，报告仍为
  `ENVIRONMENT_PREFLIGHT_COMPLETE_NOT_HARDWARE_VALIDATED` 和 `hardware_validation=NOT_RUN`。
- “文件存在”“进程启动”“设备节点有权限”不能写成相机、串口或硬件验证通过。
- 没有两类真实 golden frame、CRC 源码定位、端序/ABI、设备和固件证据时，不得修改
  `protocol_new.hpp`、CRC、parser、帧结构或帧长度。
- 任一安全默认值不满足、证据来源不明或停止条件触发时立即停止；不打开串口，不运动云台，
  不发射，回到离线检查。

## 回退

运行时回退是终止只读预检进程；它不持有设备句柄且不修改系统。仓库回退应 revert 本证据包
对应提交，删除范围仅限 `tools/orin_hardware_evidence/`、本文和两个空白模板，不需要触碰任何
协议、串口、ROS 或自瞄核心文件。

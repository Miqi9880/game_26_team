# VisionData / RobotCtrlData golden frame 电控取证模板

> 本模板用于向电控索取证据，本身不是电控证据。必须粘贴由真实设备、指定固件和可说明的采集
> 方式得到的完整原始十六进制字节，不得手工拼帧、用仓库测试向量冒充或从字段值反推。填写后
> 保存到团队批准的受控证据目录；仓库只保留空白模板。

## 1. 请求与来源

| 字段 | 内容 |
|---|---|
| 证据记录 ID | `<必填>` |
| 请求人 / 电控提供人 / 复核人 | `<必填>` |
| 采集日期、时间、时区 | `<必填>` |
| 通信链路 | `CDC USB` |
| 已确认波特率/USB line coding | `<UNKNOWN 或附证据的值；代码中的 115200 不是确认依据>` |
| 上位机 commit SHA | `<完整 SHA>` |
| 电控固件仓库与 commit/版本 | `<完整 SHA 或可追溯发布版本>` |
| 板卡与设备 | `<型号/资产标签；不要记录设备序列号>` |
| 编译器 / flags / packing | `<必填>` |
| 采集工具与版本 | `<逻辑分析仪/USB 抓包/固件原始 buffer dump；必填>` |
| 采集点与方式 | `<CRC 前/驱动前/线缆侧等；说明是否包含完整帧>` |
| 原始采集文件 | `<受控位置、文件名、SHA-256；不得提交敏感文件>` |

## 2. 已知边界（用于核对，不用于生成帧）

| 数据 | 方向 | 命令号 | payload | 当前主线完整帧 |
|---|---|---:|---:|---|
| `VisionData` | 下位机到上位机 | `VISION_ID=0x0105` | 47 bytes | 58 bytes，CRC16 后为 `0D 0A` |
| `RobotCtrlData` | 上位机到下位机 | `CHASSIS_CTRL_CMD_ID=0x0102` | 26 bytes | 35 bytes，CRC16 后不追加 `0D 0A` |

`RobotCtrlData` 35-byte 边界对应已合并的
[Fix 0x0102 RobotCtrl frame length without tail #13](https://github.com/Miqi9880/game_26_team/pull/13)。
该 PR 不是现场固件一致性的证明。若实采与表中边界不同，停止联调并提交差异证据，不修改
`protocol_new.hpp`、CRC、parser、帧结构或长度来迎合单帧结果。

## 3. VisionData golden frame（至少一帧）

| 字段 | 实采内容 |
|---|---|
| 方向 | `下位机 -> 上位机` |
| 完整帧长度 | `<十进制 bytes；预期边界 58>` |
| 完整原始 hex | `<从第 1 byte 到最后 0D 0A；空格分隔，禁止省略>` |
| header 原始 hex | `<必填>` |
| `data_length` 原始 hex / 解码值 | `<原始 bytes>` / `<值与语义>` |
| `seq` 原始 hex / 解码值 | `<原始 byte>` / `<值>` |
| header CRC8 原始值 / 校验结果 | `<hex>` / `<MATCH/MISMATCH + 工具输出>` |
| 命令号原始 hex / 解码值 | `<原始 bytes>` / `<0x0105；注明字节序>` |
| payload 原始 hex / 长度 | `<连续 47 bytes>` / `<实测>` |
| payload 字段解码 | `<逐字段：原始 bytes、类型、值、单位；未知语义写 UNKNOWN>` |
| CRC16 原始 hex | `<必填；注明两个字节顺序>` |
| CRC16 覆盖范围 | `<起止 byte offset；不得猜测>` |
| CRC16 源码追溯 | `<固件 repo、commit、文件、函数、构建版本、源码 SHA-256>` |
| 独立 CRC 校验 | `<工具/版本/算法参数、输入文件 SHA-256、期望/实际、MATCH/MISMATCH>` |
| 帧尾 | `<实采最后两字节；预期 0D 0A>` |
| 采集时间与原始日志位置 | `<必填>` |

## 4. RobotCtrlData golden frame（至少一帧）

安全要求：用于取证的控制值必须经现场负责人批准；本模板阶段默认只接受
`serial_enabled=false`、`dry_run=true`、`allow_fire=false`、`fire_command=0` 的离线记录。
本任务不发送该帧到真实设备。

| 字段 | 实采内容 |
|---|---|
| 方向 | `上位机 -> 下位机` |
| 完整帧长度 | `<十进制 bytes；预期边界 35>` |
| 完整原始 hex | `<从第 1 byte 到 CRC16 最后 1 byte；空格分隔，禁止省略>` |
| header 原始 hex | `<必填>` |
| `data_length` 原始 hex / 解码值 | `<原始 bytes>` / `<值与语义>` |
| `seq` 原始 hex / 解码值 | `<原始 byte>` / `<值>` |
| header CRC8 原始值 / 校验结果 | `<hex>` / `<MATCH/MISMATCH + 工具输出>` |
| 命令号原始 hex / 解码值 | `<原始 bytes>` / `<0x0102；注明字节序>` |
| payload 原始 hex / 长度 | `<连续 26 bytes>` / `<实测>` |
| payload 字段解码 | `<逐字段：原始 bytes、类型、值、单位>` |
| `fire_command` 原始 byte / 值 | `<hex>` / `0` |
| CRC16 原始 hex | `<必填；注明两个字节顺序>` |
| CRC16 覆盖范围 | `<起止 byte offset；不得猜测>` |
| CRC16 源码追溯 | `<固件 repo、commit、文件、函数、构建版本、源码 SHA-256>` |
| 独立 CRC 校验 | `<工具/版本/算法参数、输入文件 SHA-256、期望/实际、MATCH/MISMATCH>` |
| CRC16 后字节 | `<预期 NONE；如有任何 byte，原样记录并停止>` |
| 采集时间与原始日志位置 | `<必填>` |

## 5. 端序、ABI 与固件一致性

| 检查 | 电控证据 | 视觉侧复核 | 结果 |
|---|---|---|---|
| 多字节整数端序 | `<编译目标、文档或最小实测>` | `<独立解码>` | `<MATCH/MISMATCH>` |
| IEEE-754 float 表示与端序 | `<编译器/ABI/实测 bytes>` | `<独立解码>` | `<MATCH/MISMATCH>` |
| struct packing/alignment | `<pragma/attribute、sizeof、offsetof 构建输出>` | `<主线 SHA 对应记录>` | `<MATCH/MISMATCH>` |
| header/data_length/seq 语义 | `<固件源码路径与 commit>` | `<原始帧逐字节标注>` | `<MATCH/MISMATCH>` |
| CRC8/CRC16 算法与覆盖范围 | `<源码、commit、函数、参数>` | `<独立工具复算>` | `<MATCH/MISMATCH>` |
| 固件实际烧录版本 | `<烧录日志/版本输出的受控证据 ID>` | `<请求版本对照>` | `<MATCH/MISMATCH>` |
| CDC USB 配置 | `<固件 USB descriptor/配置证据>` | `<主机枚举证据>` | `<MATCH/MISMATCH>` |

## 6. 验收与差异处理

- [ ] 两个方向各有至少一帧完整、未裁剪的原始 hex；
- [ ] 原始文件、固件、主机 commit、采集工具和时间可追溯；
- [ ] 长度、header、`data_length`、`seq`、命令号、payload、CRC 和帧尾逐项标注；
- [ ] CRC 使用独立实现复算，结果与原始帧一致；
- [ ] 端序、float、packing、ABI 和实际烧录固件都有证据；
- [ ] `RobotCtrlData` CRC16 后确认没有 `0D 0A` 或其他尾字节；
- [ ] 复核人签字，所有 `UNKNOWN`/`MISMATCH` 已列为阻断项。

最终结论：`<ACCEPTED_EVIDENCE / REJECTED_EVIDENCE / PENDING；说明范围>`

阻断项：`<必填；无则 NONE>`

回退方法：保持 `serial_enabled=false`、`dry_run=true`、`allow_fire=false`、`fire_command=0`，不打开
串口、不运动云台、不发射；保留原始采集与 hash，回到电控/视觉双方确认。证据闭环前不得修改
协议实现，模板填写完成也不等于电控证据通过。

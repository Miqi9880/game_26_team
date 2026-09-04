# 离线自瞄证据报告工具

`tools/offline_evidence_report/auto_aim_evidence_report.py` 是一个只读、纯 Python 标准库工具，用于检查 `auto_aim_offline` 产生的 CSV，并写出 JSON 与 Markdown 证据报告：

```bash
python3 tools/offline_evidence_report/auto_aim_evidence_report.py \
  --input-csv /absolute/path/to/auto_aim.csv \
  --json-report /tmp/game26_evidence.json \
  --markdown-report /tmp/game26_evidence.md
```

工具按列名读取 CSV，不依赖固定列号；会检查必需列、重复列名、空值、非法整数/浮点数、`NaN`/`Inf`、未知 tracker 状态、非法 `target_lock`、帧/时间倒退或重复、同一帧多行、缺帧和帧内矛盾。输入异常时尽量先写出报告，再以 `PASS=0`、`WARN=2`、`FAIL=1` 退出。

CSV 输入必须是本地普通文件；FIFO、字符/块设备、目录和符号链接在打开前被拒绝。JSON/Markdown 输出同样只允许普通文件，且拒绝输入/输出别名、符号链接、硬链接和特殊文件，因此报告路径不能意外变成串口或其他设备写入目标。

报告是软件结构和数据质量证据，不是命中率、比赛成绩、云台闭环验收、硬件验证或两车对打结论。`target_lock=49` 只代表软件离线输出，不能解释为真实云台锁定。统计不修改 `auto_aim_offline`，不修改 CSV schema，不连接相机、串口、ROS、OpenVINO、Orin 或机器人。

每帧可能有多条 detection 行，因此帧级状态、selected、lock 和 fire 统计按唯一 frame 聚合；检测/PnP 统计仍按 CSV 中已有记录统计。工具不计算真实距离精度或命中率。

安全边界固定为：`serial_enabled=false`、`dry_run=true`、`allow_fire=false`、`fire_command=0`。
任何非零 `fire_command`、非零 yaw/pitch 速度或加速度、`production_ready=true`、`test_only=false`，以及
ballistic control-applied/serial-enabled/allow-fire 声明都会被拒绝为安全异常；报告本身不执行动作。

## 可选弹道诊断后缀

新 CSV 可在旧列之后追加完整的 `ballistic_*` 后缀；旧 CSV 没有这个后缀仍兼容。后缀出现时，报告检查
enabled/valid/reason、finite 数值、正飞行时间、`horizon_ns = latency_ns + flight_time_ns`、muzzle origin
assumption、`test_only=true`、`production_ready=false` 与 `ballistic_control_applied=false`。完整有效记录会被
汇总为 valid/invalid/disabled 分布、failure reason、origin、飞行时间和推荐 horizon；它们是无阻力数学
诊断，绝不是实测弹速、真实延迟、真实命中率或硬件结果。

## 单位

- 外部位置角：`degree`
- 外部角速度：`degree/s`
- 外部角加速度：`degree/s²`
- 算法内部位置角：`rad`
- 算法内部角速度：`rad/s`
- 算法内部角加速度：`rad/s²`
- `fire_command` 基线：`0`

工具只引用 CSV 已有字段，不改单位、不覆盖原数据、不生成 `RobotCtrl`。报告写入前会拒绝与输入 CSV 相同或别名的 JSON/Markdown 路径（包括符号链接和硬链接），也拒绝 JSON 与 Markdown 彼此别名。

## 可选元数据

可用 `--metadata-json` 记录经审查的 `commit`、`model_profile_id`、`model_profile_version`、`pnp_profile`、`dataset_id` 和 `source_label`。工具不会保存 token、密码、设备敏感信息、未经审查的模型/标定数据或完整个人路径。

## 在 Orin/实车前使用

在进入 Orin 或实车前，先保存原始 CSV 与标注 PNG，再在隔离环境运行报告工具，审阅所有错误/警告、`test_only`、模型/PnP profile、时间戳、锁定状态和 PnP 失败分布。只有真实模型、相机内参、装甲尺寸、camera→gimbal 外参、绝对角零点和通信安全策略分别完成评审后，才可进入硬件验证；本工具本身不会打开硬件链路。

建议将同一次运行的 CSV、PNG、JSON、Markdown、提交 SHA 和元数据放在同一归档目录。不要把 test-only fixture 包装成 production 结果。

## 已知限制与回退

- 工具不能判断检测框、关键点或标定是否真实正确，只能检查 CSV 结构和有限值。
- 它不能证明真实距离精度、命中率、开火时序、云台闭环或比赛成绩。
- 当输入 schema 扩展时，保留必需列并按列名读取；未知列会被保留但不参与推断。
- 若报告器不可用，保留原始 CSV/PNG，使用 `python3 -m py_compile` 和标准库 `unittest` 检查工具，或回退到人工审阅；不要修改生产 ROS/控制链路来绕过报告。

固定证据边界：

```yaml
software_structure_statistics_only: true
real_hit_rate_computed: false
hardware_validation: false
gimbal_closed_loop_validated: false
firing_validated: false
```

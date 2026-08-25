# 离线自瞄运行证据包与准入检查

`offline_evidence_bundle.py` 将一次 `auto_aim_offline` 离线运行整理为可复现、可归档、可答辩解释的文件包。它只读 CSV 和用户提供的离线证据，调用已有的 `auto_aim_evidence_report.analyze_csv()` 与 `build_report()`，不复制 CSV 解析逻辑。

工具只使用 Python 标准库；不导入 ROS、OpenVINO、相机 SDK、串口或机器人控制代码，不连接真实相机/Orin/云台，不生成 `RobotCtrl`，不执行 `fire_command`。

## 完整命令

在 `/home/ubuntu22/vision-study/game_26_evidence_bundle` 执行：

```bash
python3 tools/offline_evidence_report/offline_evidence_bundle.py \
  --input-csv /absolute/path/run.csv \
  --output-dir /absolute/path/evidence_bundle \
  --metadata-json /absolute/path/run_metadata.json \
  --camera-intrinsic-report /absolute/path/camera_intrinsic_report.yaml \
  --model-profile /absolute/path/model_profile.yaml \
  --pnp-config /absolute/path/pnp_config.yaml \
  --annotated-dir /absolute/path/annotated \
  --producer-command-file /absolute/path/producer_command.txt \
  --mode evidence_only
```

`--input-csv` 和 `--output-dir` 是构建模式的必需参数；metadata 和其他证据是可选的。输出目录不存在时自动创建。`--mode evidence_only` 允许缺少真实硬件材料，但会写出明确 WARN；`--mode strict` 要求 metadata、相机内参引用、model profile 和 PnP config 均存在且哈希通过，缺失即 FAIL。两种模式都要求 CSV 和生成的 JSON/Markdown 报告完整，CSV 本身 FAIL 或安全异常时包为 FAIL；strict 模式下 CSV WARN 也会升级为 FAIL。

状态码固定为：`PASS=0`、`WARN=2`、`FAIL=1`。即使 CSV 缺失、损坏或输入异常，工具仍尽量写出 `manifest.json`、`summary.md`、`csv_report.json` 和 `csv_report.md`。

已有包可以只校验 manifest：

```bash
python3 tools/offline_evidence_report/offline_evidence_bundle.py \
  --output-dir /absolute/path/evidence_bundle \
  --verify-manifest /absolute/path/evidence_bundle/manifest.json
```

## 输出结构

最小包包含：

```text
evidence_bundle/
├── manifest.json
├── summary.md
├── csv_report.json
├── csv_report.md
└── input/auto_aim.csv
```

提供相应参数时还会复制：`run_metadata.json`、`camera_intrinsic_report.yaml`、`model_profile.yaml`、`pnp_config.yaml`、`producer_command.txt` 和 `annotated/` 下的 PNG 文件。原始 CSV 和配置文件以文件内容复制并计算哈希；producer command 会先脱敏再保存。

## Manifest schema

`manifest.json` 是稳定排序的 JSON 对象：

```yaml
schema_version: 1
status: PASS | WARN | FAIL
mode: evidence_only | strict
bundle_id: run_id_or_bundle_basename
input_csv_name: basename_only
metadata: allow-listed_run_metadata
required_roles: [input_csv, csv_report_json, csv_report_markdown]
strict_roles: [run_metadata, camera_intrinsic_report, model_profile, pnp_config]
unhashed_files: [manifest.json]
artifacts:
  - role: input_csv
    path: input/auto_aim.csv
    size_bytes: 1234
    sha256: 0123...
diagnostics:
  errors: []
  warnings: []
safety_boundary:
  serial_enabled: false
  dry_run: true
  allow_fire: false
  fire_command: 0
evidence_boundary:
  software_structure_statistics_only: true
  real_hit_rate_computed: false
  hardware_validation: false
  gimbal_closed_loop_validated: false
  firing_validated: false
```

artifact `role` 和 POSIX 相对 `path` 必须唯一；路径不能是绝对路径、Windows drive/UNC 路径或包含 `..`，也不能指向 bundle 外的 symlink。文件大小和 SHA-256 每次校验。annotated 文件使用 `annotated_png:<relative-path>` 角色。

`manifest.json` 是索引文件，不能对自己做原始字节 SHA-256 自引用，所以在 `unhashed_files` 中明确列出；`summary.md` 作为 artifact 记录并哈希，但它的表格排除自身，避免循环依赖。构建两次时相同输入、metadata 和证据文件会得到稳定的 manifest/artifact 排序和内容。

## Metadata 与脱敏

允许记录：`run_id`、`commit`、`dataset_id`、`model_profile_id`、`model_profile_version`、`pnp_profile`、`source_label`、`run_command`。未知字段、token、password、secret、API key 等被忽略或脱敏；命令中的绝对路径仅保留 `<path>/basename`，不会执行命令。manifest 只记录输入源 basename，不记录完整个人目录。

## 准入与安全边界

包始终展示：

```yaml
production_ready: false
hardware_validation: false
gimbal_closed_loop_validated: false
firing_validated: false
real_hit_rate_computed: false
serial_enabled: false
dry_run: true
allow_fire: false
fire_command: 0
```

CSV 报告中的非零 `fire_command`、`test_only=false`、报告 FAIL、声明硬件/命中率/闭环已验证，均会产生 FAIL 或安全异常。`target_lock=49` 只是离线软件字段，不是真实云台锁定。低重投影误差只说明该 CSV/PnP 假设的结构统计，不能推出真实距离精度、命中率、比赛成绩或两车对打表现。

`camera_intrinsic_report` 若声明 `profile: evidence_only`/`test_only` 却又声明 `production_ready: true`，工具会 FAIL，绝不会自动升级正式标定。工具不会生成或修改 production PnP YAML、相机正式标定、camera→gimbal 外参、绝对角零点、RobotCtrl 或开火命令。

## 归档、缺失和回退

一次运行应把原始 CSV、标注 PNG、CSV JSON/Markdown、相机 evidence-only 报告、model profile、PnP config、metadata、producer command、manifest 和 summary 一起保存，并在答辩时提供提交 SHA 与各文件 SHA-256。缺失文件、不可读文件、manifest 格式错误、重复 role/path、路径穿越、symlink/hardlink、大小或哈希不一致都要先修复或明确记录，不能把 WARN 包装成 production 结果。

如果工具不可用，保留原始 CSV/PNG 和已有 PR #16 报告，人工核对 manifest 中的相对路径与 SHA-256；不要修改 ROS/控制链路绕过准入检查。功能回退使用：

```bash
git revert <功能提交SHA>
```

该工具不验证真实模型、正式相机标定、Orin、串口、云台闭环、发射时序或比赛成绩；这些事项仍需独立的硬件和裁判证据。

# 离线模型 Profile 审核与回放准入

`tools/auto_aim_qualification/auto_aim_qualification.py` 是一个只读的统一准入门。它不连接 ROS、OpenVINO、相机 SDK、串口、云台或机器人，也不执行 `fire_command`。现有 C++ `load_model_profile()`、`load_pnp_configuration()` 和 PR #16/#17 工具仍是运行时/证据链的规范；Python 审核只在进入离线报告前把缺失、路径、hash、语义和版本缺口显式化。

## 工作流

```text
model + model_profile + PnP profile
  → profile/PnP audit
  → existing auto_aim_offline CSV (if available)
  → PR #16 JSON/Markdown report
  → PR #17 evidence bundle/manifest
  → qualification.json / qualification.md
```

没有正式模型或 OpenVINO 运行资产时，工具不会伪造 detector 回放；它记录 `pipeline_execution_unavailable`，并允许用仓库内的 test-only profile、PnP fixture 和合成 CSV 复现准入逻辑。

## Evidence-only（当前阶段）

必须显式传入 `--allow-test-only` 才允许 `profile: test_only`。示例：

```bash
# 执行目录：/home/ubuntu22/vision-study/game_26_model_qualification
python3 tools/auto_aim_qualification/auto_aim_qualification.py \
  --mode evidence_only --allow-test-only \
  --model-profile src/auto_aim_ros2/test/data/model_profile_test.yaml \
  --pnp-config src/auto_aim_ros2/test/data/pnp_test_config.yaml \
  --input-csv tools/offline_evidence_report/fixtures/normal.csv \
  --metadata-json tools/offline_evidence_report/fixtures/bundle/normal_metadata.json \
  --evidence-bundle /tmp/game26_qualification_bundle \
  --output-json /tmp/game26_qualification.json \
  --output-markdown /tmp/game26_qualification.md
```

该模式最多得到 `WARN`，绝不会写出 production-ready；CSV 结构、安全异常或 manifest/hash 错误仍为 `FAIL`。

## Strict

`--mode strict` 要求正式 model profile、实际模型文件、profile 中声明的模型 SHA-256、正式 PnP/K-D/装甲尺寸/camera→gimbal 外参、完整 metadata、PR #16 报告和 PR #17 manifest。任何 test-only、路径或 hash 不一致、CSV `fire_command != 0`、时间戳异常、报告/manifest 缺失都会 FAIL。工具不能生成正式 PnP、外参、绝对角零点、RobotCtrl 或开火参数。

## 固定安全边界

所有报告始终包含：

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
yaw_vel: 0
pitch_vel: 0
yaw_acc: 0
pitch_acc: 0
```

状态码沿用现有证据工具：`PASS=0`、`WARN=2`、`FAIL=1`。输出中的模型路径只保留 basename；metadata 和 producer command 不能包含 token、密码或个人目录。

## 回退

本功能只新增 `tools/auto_aim_qualification/` 和对应文档/测试；回退使用：

```bash
git revert <功能提交SHA>
```

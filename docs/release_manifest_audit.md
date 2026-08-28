# 软件候选版发布 manifest 审计

`release_manifest_audit` 是只读、SDK-independent 的离线工具。它不启动
ROS，不连接 MVS/OpenVINO/Orin/CDC，不创建真实 `/Robot_ctrl_data`
publisher，不写串口，不运动云台，也不触发开火。它只读取调用方显式声明的
JSON 证据路径。

## 输入约定

配置文件是 schema v1 JSON：

```json
{
  "schema_version": 1,
  "candidate": {
    "head": "<40 hex SHA>",
    "main_baseline": "<40 hex SHA>",
    "branch": "feat/release-manifest-audit",
    "worktree_clean": true
  },
  "sources": [
    {"id":"release-smoke", "kind":"release_smoke", "path":"/abs/smoke-report.json",
     "ctest_xml":"/abs/Testing/TAG/Test.xml"},
    {"id":"ros-e2e", "kind":"ros_e2e", "path":"/abs/e2e-report.json",
     "absence_status":"NOT_VERIFIED"}
  ]
}
```

`path` 必须由调用方提供，工具不会按文件名猜测版本或寻找替代文件。可选的
`sha256`（64 位十六进制）会与实际文件 hash 比较；缺失/空文件只有在显式
`absence_status`（仅 `UNAVAILABLE`、`NOT_RUN` 或 `NOT_VERIFIED`）时才会被记录为不可用，
否则为 FAIL；缺失或空的来源/产物绝不能声明为 `PASS`。

任何带有 `counts`/`cases` 的 CTest-backed 报告都必须在来源声明中提供
`ctest_xml`。审计会读取该 XML，并 fail-closed 地比较总案例数与
`PASS`、`FAIL`、`NOT_RUN` 统计；缺失、损坏、无 `Test` 记录或统计不一致均为 FAIL。
顶层五态还必须与 `counts` 和每个 `case.status` 的保守聚合结果一致（优先级为
`FAIL`、`NOT_VERIFIED`、`NOT_RUN`、`UNAVAILABLE`、`PASS`）；任何矛盾都会 FAIL。

运行：

```bash
release_manifest_audit --config /abs/release-input.json \
  --output-dir /abs/new-release-manifest
```

输出目录必须不存在。输出为 `release-manifest.json`、`SUMMARY.md` 和
`SHA256SUMS`；重复运行请使用新目录。canonical JSON 使用排序后的对象键，
不写生成时间，因此与路径、输入内容无关的日志或输入数组顺序不会改变结果。

## fail-closed 规则

- candidate HEAD、main baseline 必须是 40 位 SHA，工作区必须声明干净；
- 来源 JSON 必须是 schema v1、状态和 CTest XML 的总数/统计一致，Git SHA 与候选
  一致，声明的 hash 与实际 hash 一致；
- `production_ready`、硬件验证、开火、命中率等任何生产性声明为 true 都失败；
- 证据来源必须保持 `synthetic=true`、`test_only=true`、`production_ready=false`；
- 安全字段必须保持 `serial_enabled=false`、`dry_run=true`、`allow_fire=false`，
  以及 fire/yaw/pitch 的命令和速度/加速度均为零；
- #34 ROS 消息级 E2E 必须额外提供
  `node_liveness.alive_during_sampling=true`、`expected_exit_code` 和
  `observed_exit_code` 且二者相等。缺失或矛盾会是 `NOT_VERIFIED`，不会计入
  发布 PASS；
- 缺模型、标定、外参或硬件只能传播为 `UNAVAILABLE`、`NOT_RUN` 或
  `NOT_VERIFIED`，不能折算为 PASS。

既有 Python evidence/qualification 工具的 `WARN` 会传播为
`NOT_VERIFIED`，不被伪装成 PASS。

退出码为 `0=PASS`、`1=FAIL`、`2=NOT_VERIFIED/其他不可验证状态`。

审计通过不等于真实相机、Orin、正式模型/标定、CDC、机器人、云台、开火、真实
延迟、命中率或比赛性能已经验证。失败时保留原始来源和日志，修复输入后用新
输出目录重跑；回退使用 `git revert <功能提交SHA>`。

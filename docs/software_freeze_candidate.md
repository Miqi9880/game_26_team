# Software freeze candidate gate

`tools/software_freeze/software_freeze_gate.py` is a read-only admission check for the software-freeze candidate. It does not add or alter detector, PnP, Tracker, Selector, Predictor, Aimer, ballistic, protocol, camera, serial, ROS control, gimbal, or firing behavior. It executes commands without a shell (apart from the explicitly documented `source`/`colcon` wrappers), records exit codes and sanitized output, and supports injected command results for deterministic tests.

The live mode is a complete evidence collector. The explicit-input mode
(`--input-manifest`) is the final admission interface when outputs were
already produced by a separate run: it consumes only caller-declared paths,
never guesses by filename, and emits a versioned JSON/Markdown/SHA-256 bundle.

## Decision contract

The report has a stable schema (`software-freeze-candidate`, version `1`) and uses only `PASS`, `FAIL`, `BLOCKED`, `UNAVAILABLE`, `NOT_RUN`, and `NOT_VERIFIED` for individual checks. The admission status is one of:

- `READY_CANDIDATE`: every available software check passed; hardware/formal-artifact gaps are listed explicitly as `UNAVAILABLE` or `NOT_VERIFIED`.
- `NOT_VERIFIED`: no software regression was found, but a required software evidence input or dependency was unavailable/not run.
- `BLOCKED` (or the machine-readable `FAIL` status on an individual input): a software check, safety field, hash, schema, liveness record, or consistency assertion failed.

No historical PR number is a hard-coded admission prerequisite. GitHub state, when queried by the live mode, is informational only; the candidate SHA, main baseline, branch, clean worktree, and explicit evidence paths are normative.

## Commands and evidence

Run from a fresh worktree created from the actual remote main. Set `FREEZE_WORKTREE` to that worktree before invoking the gate; the value is intentionally not committed in reports or source. The gate records `rosdep check --from-paths src --ignore-src`, the complete eight-package colcon build/test matrix (plus standalone `orin_hardware_evidence` CMake/CTest), `git diff --check`, Python qualification/evidence tests, release-smoke and ROS message-E2E wrapper commands/results, all installed CLI `--help` probes, camera preflight as `NOT_VERIFIED` unless a separate approved hardware run supplies evidence, ROS-resolved `auto_aim_scenario_benchmark --help`, and two `--scenario all --seed 260033` runs. Scenario records must include `benchmark.csv`, `summary.json`, and `summary.md` sizes and SHA-256 values from both new output directories. Existing or linked output directories are rejected before writing.

The ROS message-level publisher/node matrix is a separate installed-runner
check; the normal gate covers its package build and `report_test`, then
records the explicit wrapper report (including node liveness). When that
functional matrix is required, run `src/auto_aim_ros_e2e/scripts/run_ros_message_e2e.sh`
with a new output root and retain its report as additional evidence.

Use a new external directory for the report bundle (the live collector will
return `NOT_VERIFIED` when rosdep, hardware, or formal evidence is absent;
it will return `BLOCKED` for a failed software check):

```bash
python3 tools/software_freeze/software_freeze_gate.py \
  --repo-root "$FREEZE_WORKTREE" \
  --output-dir /tmp/game26-freeze-report
```

The output contains deterministic JSON, Markdown, `manifest.json`, and `SHA256SUMS`; it never overwrites a sentinel or follows a link. Generated reports, logs, model weights, calibration data, device serial numbers, and personal paths are not committed.

For a final freeze, prefer the explicit-input mode. Create a JSON file (schema
`software-freeze-inputs`, version `1`) with no implicit filename discovery:

```json
{
  "schema": "software-freeze-inputs",
  "schema_version": 1,
  "candidate": {
    "head": "<40-hex-candidate-sha>",
    "main_baseline": "<40-hex-origin-main-sha>",
    "branch": "codex/software-freeze",
    "worktree_clean": true
  },
  "required_kinds": ["release_smoke", "ros_e2e", "scenario_benchmark"],
  "inputs": [
    {"id": "release-smoke", "kind": "release_smoke", "path": "/abs/smoke-report.json",
     "sha256": "<64-hex-sha256>"},
    {"id": "ros-e2e", "kind": "ros_e2e", "path": "/abs/ros-message-e2e-report.json",
     "sha256": "<64-hex-sha256>"},
    {"id": "scenario", "kind": "scenario_benchmark", "path": "/abs/summary.json",
     "sha256": "<64-hex-sha256>", "ctest_xml": "/abs/Testing/TAG/Test.xml",
     "ctest_xml_sha256": "<64-hex-sha256>"}
  ],
  "artifacts": [
    {"role": "model_xml", "path": "/abs/model.xml", "absence_status": "NOT_VERIFIED"},
    {"role": "model_bin", "path": "/abs/model.bin", "absence_status": "NOT_VERIFIED"},
    {"role": "model_profile", "path": "/abs/model-profile.yaml", "absence_status": "NOT_VERIFIED"},
    {"role": "pnp_config", "path": "/abs/pnp.yaml", "absence_status": "NOT_VERIFIED"},
    {"role": "calibration_manifest", "path": "/abs/calibration.json", "absence_status": "NOT_VERIFIED"}
  ]
}
```

Run it with `--input-manifest` (aliases `--inputs-json` and
`--manifest-config` are accepted):

```bash
python3 tools/software_freeze/software_freeze_gate.py \
  --input-manifest /abs/software-freeze-inputs.json \
  --output-dir /tmp/game26-freeze-report
```

Every declared input and artifact is recorded with its resolved path, byte
size, SHA-256, schema, status, and failure reason. Every present input and
artifact must declare the expected SHA-256; a missing declaration is a
validation failure. Missing paths are accepted
only with an explicit `absence_status` of `UNAVAILABLE`, `NOT_RUN`, or
`NOT_VERIFIED`; the tool never searches a directory or substitutes a file
whose name happens to match. CTest XML totals, per-case names/statuses,
candidate and baseline
SHA references, model XML/BIN/profile, PnP, calibration-manifest hashes,
ROS-E2E `node_liveness`, and safe control fields are compared fail-closed.
`skip_ctest` is not an accepted bypass. If a producer has CTest-shaped
`counts`/`cases`, the corresponding XML (and its byte SHA-256) must be named
explicitly and agree case-by-case.

负向 evidence 只能通过显式、可复核的拒绝契约计入软件准入：输入声明必须同时
给出 `negative_test=true`、`expected_failure=true`、固定的
`expected_diagnostic_code="calibration_promotion"`、非零
`expected_exit_code` 和 `fixture_sha256`。来源 JSON 必须声明同一个 fixture hash、
实际非零退出码、`production_claim_rejected=true`、所有状态别名均为 `FAIL`，并且
diagnostics 只能包含一个同码错误且没有额外 warning；任何缺字段、额外错误或
真实软件失败仍为 `BLOCKED`。

## Safety and evidence boundary

Every report preserves:

```text
serial_enabled=false
dry_run=true
allow_fire=false
fire_command=0
yaw_vel=0
pitch_vel=0
yaw_acc=0
pitch_acc=0
```

The gate only inspects command results and evidence files. It does not open a serial port, camera, Orin, robot, or gimbal, does not create a control publisher, does not send a frame, move hardware, or fire. Recursive evidence scanning rejects `production_ready=true`, `test_only=false`, hardware/closed-loop/firing claims, non-zero fire or motion fields, serial/allow-fire claims, and ballistic control application.

Model artifacts, formal K/D and camera-to-gimbal calibration, camera SDK, Orin, CDC golden frames, gimbal closed loop, and firing are explicitly `NOT_VERIFIED` or `UNAVAILABLE`. These are non-blocking hardware gaps for a software-only candidate, never production PASS claims.

## Tests and rollback

The unit suite uses fake observations/results and covers clean and dirty worktrees, stale HEAD, unavailable dependencies, a single failed check, flaky ROS rounds, scenario byte mismatch, production claims, non-zero fire/motion, unrun checks, hardware-unverified state, reproducible manifests, existing directories, and symlinks. Run it with:

```bash
python3 -m unittest discover \
  -s tools/software_freeze/test -p 'test_*.py' -v
python3 -m py_compile tools/software_freeze/software_freeze_gate.py
```

本次受限 diff 只覆盖冻结门禁、ROS E2E liveness、发布审计及对应文档/测试；未改动
核心算法、协议、真实串口或硬件路径。回退提交使用 `git revert <commit-sha>`；不要使用
reset、checkout 或 clean。

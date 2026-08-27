# Software candidate install and offline release smoke

This procedure validates a copied ROS 2 install tree without a real camera,
MVS SDK, Orin, CDC serial device, robot, gimbal, firing mechanism, production
model, or formal calibration. A `PASS` proves only the software cases listed in
the generated report. It is not hardware, accuracy, latency, closed-loop, or
firing validation.

## Scope and installed resources

The clean colcon build covers these packages:

- `auto_aim_interfaces`
- `serical_device_ros2`
- `auto_aim_ros2`
- `auto_aim_tools`
- `hik_camera`
- `auto_aim_release_smoke`

ROS executables are installed below each package's `lib/<package>` directory.
The camera `launch` and example `config` files remain in
`share/hik_camera`. The release-smoke package installs the repository docs,
qualification/evidence tools, and explicitly test-only model/PnP fixtures in
`share/auto_aim_release_smoke`. Production model XML/BIN, production model
profiles, formal K/D, and formal camera-to-gimbal extrinsics are deliberately
not shipped by this repository and must be supplied and reviewed separately.

The installed example camera calibration is unverified and cannot satisfy the
formal camera contract. The installed model and PnP fixtures are marked
`test_only`; they require explicit test-only flags and cannot be promoted to
production.

## Prerequisites

Use Ubuntu 22.04 with ROS 2 Humble, `colcon`, CMake, a C++17 compiler, OpenCV,
OpenSSL, yaml-cpp, and the ROS dependencies declared in each `package.xml`.
`rosdep` is checked when initialized. An uninitialized or missing `rosdep` is
reported as `UNAVAILABLE`; it is never represented as dependency `PASS`.

OpenVINO and the MVS SDK are optional for this SDK-independent smoke. Their
absence is reported as `UNAVAILABLE`. Do not install fake SDK libraries or
create fake device files to obtain a more favorable report.

## Reproducible clean run

Run from a clean feature worktree based on the actual current main SHA. The
output root must not exist; reuse is rejected before any mutation.

```bash
cd /home/ubuntu22/vision-study/game_26_issue_31
source /opt/ros/humble/setup.bash

BASELINE=9de02ae662dfecec02ad4701beb626996554935d
OUTPUT=/tmp/game26-release-smoke-$(git rev-parse --short HEAD)

bash src/auto_aim_release_smoke/scripts/run_release_smoke.sh \
  --workspace-root "$PWD" \
  --output-root "$OUTPUT" \
  --baseline "$BASELINE"
```

The wrapper performs a normal copied install, not `--symlink-install`, using
only `$OUTPUT/build`, `$OUTPUT/install`, and `$OUTPUT/log`. It runs:

```text
rosdep check --from-paths src --ignore-src
colcon build ... -DBUILD_TESTING=ON
colcon test ...
colcon test-result --verbose
cmake/build/ctest for tools/orin_hardware_evidence
installed auto_aim_release_smoke
```

Reports are written to `$OUTPUT/report/smoke-report.json` and
`$OUTPUT/report/smoke-report.md`. Per-case command output remains under
`$OUTPUT/report/logs`. The report records the main baseline, tested commit,
environment, safe defaults, status counts, exact cases, and unverified items.

## Offline evidence sequence

The reproducible offline sequence is:

1. Audit the installed test-only model profile and confirm that a missing
   OpenVINO runtime or model is `UNAVAILABLE`/fail-closed, never `PASS`.
2. Load only the installed PnP `test_only` fixture with explicit test-only
   authorization. Missing PnP configuration must fail before inference.
3. Run `auto_aim_offline` only when a reviewed local model, profile, video, and
   PnP input are supplied. Missing model, calibration, or input fails closed.
4. Treat Tracker/Selector output as offline evidence. Predictor and ballistic
   diagnostics remain diagnostic-only and cannot alter the Aimer or control
   path.
5. Save CSV and annotated PNG files to a new output path.
6. Run the installed `auto_aim_evidence_report.py`,
   `offline_evidence_bundle.py`, and `auto_aim_qualification.py` to produce and
   verify JSON/Markdown evidence.

The smoke uses the existing evidence CSV parser through the installed Python
tools. It does not reimplement that parser in C++ and does not change Tracker,
Selector, Predictor, Ballistic, or Aimer semantics.

## Status and failure interpretation

- `PASS`: an available software contract ran and matched its expected result.
- `FAIL`: an available required software contract regressed or an unsafe state
  was observed.
- `UNAVAILABLE`: an optional SDK/runtime/environment was absent, so that case
  could not be validated.
- `NOT_RUN`: a case was intentionally skipped, normally because it would
  require unavailable hardware.
- `NOT_VERIFIED`: no admissible evidence exists for the stated hardware or
  formal artifact claim.

Expected fail-closed fixtures return nonzero and still count as a smoke case
`PASS` only when the diagnostic and absence of side effects are verified. A
missing dependency that prevents reaching the intended runtime check remains
`UNAVAILABLE`.

The enforced offline defaults are:

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

The smoke uses an isolated ROS domain and never starts the serial or real
camera nodes. It checks that offline CLIs do not create a
`/Robot_ctrl_data` publisher. Normal stop, repeated SIGINT, timeout/failure,
and forced exit cases must leave no live ROS/test process group.

## Stop, failure recovery, and rollback

Press `Ctrl-C` once to stop the wrapper. Do not connect hardware to retry a
software-only failure. Preserve the output root and inspect the relevant
colcon or per-case log. Use a new output root for every rerun; never delete or
overwrite an old evidence directory to make a result pass.

If the smoke process was externally killed, verify that no process from its
isolated `ROS_DOMAIN_ID` remains before rerunning. The automated lifecycle
cases perform SIGINT/SIGKILL cleanup and report a residual group as `FAIL`.

Rollback this feature through its reviewable commit:

```bash
git revert <release-smoke-commit-sha>
```

Do not use a reset or clean operation in a dirty development worktree. An
offline smoke `PASS` does not mean that MVS SDK, real camera, Orin, CDC serial,
robot, gimbal motion, firing, formal model, or formal calibration passed.

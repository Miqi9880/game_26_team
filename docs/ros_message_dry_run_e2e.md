# ROS message-level dry-run E2E regression

This SDK-independent suite publishes synthetic ROS messages into the existing
read-only input preflight and the installed `AutoAimNode`. It observes the
existing RobotCtrl topic boundary. It never starts the MVS camera node, serial
sender, robot, gimbal, or firing path, and it does not duplicate a production
node.

A suite `PASS` is only message-level dry-run evidence. It is not evidence for a
real camera, MVS SDK, Orin, formal model or calibration, CDC serial, robot,
gimbal motion, firing, accuracy, latency, or competition performance.

## Fixed contract

Every case uses a unique run id and remapped topic root in an isolated
`ROS_DOMAIN_ID`. The valid synthetic contract is:

| Field | Value |
|---|---|
| Image topic/type | `/image_raw`, `sensor_msgs/msg/Image` |
| CameraInfo topic/type | `/camera_info`, `sensor_msgs/msg/CameraInfo` |
| QoS | SensorDataQoS: best effort, volatile, keep last, depth 5 |
| Image | `rgb8`, width 4, height 3, step 12, data size 36 |
| Header | `camera_optical_frame`, fixed nonzero synthetic timestamp sequence |
| CameraInfo | matching dimensions/stamp/frame, finite K/D, positive fx/fy |
| RobotCtrl | existing `/Robot_ctrl_data` publisher, uniquely remapped |

The C++ fixture publishes Image, CameraInfo, and bookkeeping-only Vision
messages. It is a test-only publisher and RobotCtrl subscriber. It has no
connection to `robot_ctrl_main` or any real sender. The preflight remains
subscription-only and is explicitly checked not to publish RobotCtrl.

All AutoAimNode launches set:

```text
serial_enabled=false
dry_run=true
allow_fire=false
```

Every observed safe output must have `fire_command=0`, `yaw_vel=0`,
`pitch_vel=0`, `yaw_acc=0`, and `pitch_acc=0`. Target lock is checked against
the existing interface semantics: 49 is locked and 50 is unlocked. A valid
mock target may lock while all fire and derivative fields remain inhibited;
invalid input and watchdog cases that use mock must return to unlocked.

## Coverage

The full matrix repeats every runnable software case five times with seed
`260033`:

- valid rgb8 with matching finite CameraInfo on null and mock backends;
- missing CameraInfo, dimensions mismatch, K NaN, D Inf, and zero focal length;
- empty image, zero dimensions, unsupported encoding, short stride, and short data;
- missing, mismatched, rolled-back, and duplicate timestamps;
- wrong topic, wrong QoS, and unsupported launch parameter;
- no input, input watchdog expiry, reordered valid delivery, temporary
  occlusion/reacquisition, and continuous frame interruption;
- natural stop, repeated SIGINT, runner timeout cleanup, and SIGKILL cleanup;
- missing model XML/BIN, model profile, test-only PnP/calibration, and offline
  video input;
- available null/mock state and unavailable formal `offline_reference` state.

Each case records its input summary, expected and actual state, diagnostic,
node and preflight exit codes, topic/publisher observation, control count,
safety fields, target-lock summary, cleanup result, and input/output SHA-256.
`PASS`, `FAIL`, `UNAVAILABLE`, `NOT_RUN`, and `NOT_VERIFIED` are distinct.

## Reproducible build and run

Run from the clean issue worktree on Ubuntu 22.04 with ROS 2 Humble, C++17,
colcon, OpenCV, OpenSSL, yaml-cpp, and the dependencies declared by the ROS
packages. The output root must not exist.

```bash
cd /home/ubuntu22/vision-study/game_26_issue_33
source /opt/ros/humble/setup.bash

BASELINE=28dc6e0021f16fa6675235eaed5042c43d7b011f
OUTPUT=/tmp/game26-issue33-$(git rev-parse --short HEAD)-$(date +%s)

bash src/auto_aim_ros_e2e/scripts/run_ros_message_e2e.sh \
  --workspace-root "$PWD" \
  --output-root "$OUTPUT" \
  --baseline "$BASELINE" \
  --rounds 5
```

The wrapper performs this sequence with separate build/install/log paths:

```text
colcon build -> source install -> colcon test -> colcon test-result --verbose
-> start fake publishers + preflight + real AutoAimNode
-> run valid/invalid fixtures -> observe graph/diagnostics/RobotCtrl
-> bounded stop and process-group cleanup -> JSON/Markdown/SHA-256 reports
```

To run only the installed C++ E2E reporter after a build:

```bash
source /opt/ros/humble/setup.bash
source "$OUTPUT/install/setup.bash"
export ROS2CLI_NO_DAEMON=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=133

"$OUTPUT/install/auto_aim_ros_e2e/lib/auto_aim_ros_e2e/auto_aim_ros_e2e" \
  --install-base "$OUTPUT/install" \
  --output-dir "$OUTPUT/report-only-new" \
  --baseline "$BASELINE" \
  --commit "$(git rev-parse HEAD)" \
  --rounds 5 \
  --seed 260033
```

Reports are `ros-message-e2e-report.json`, `ros-message-e2e-report.md`, and
`SHA256SUMS`. The reports list every input/output created before report
rendering; `SHA256SUMS` additionally hashes both reports and excludes only
itself to avoid a recursive hash. Verify hashes from the report directory:

```bash
cd "$OUTPUT/report"
sha256sum -c SHA256SUMS
```

## Failure, stop, and cleanup

Press Ctrl-C once. The runner forwards termination to only the process groups
it created, escalates to SIGKILL within bounded time, reaps them, and never
kills unrelated ROS/DDS processes. It uses `ROS2CLI_NO_DAEMON=1`, localhost
discovery, a unique domain, unique topics, and a private ROS home/log path.

On a first case failure, the original directory, log, exit code, and report
remain untouched. The runner repeats that case once under
`reruns/round-N/<case>` with a new run id. A changed normalized status summary
is marked non-deterministic and `FAIL`. Always use a new top-level output path
for another suite run; existing output paths are rejected rather than
overwritten.

Common failures include an un-sourced install, reused output directory,
missing ROS dependency, DDS discovery delay, unexpected publisher, changed
QoS, invalid model/profile/PnP path, or a residual process group. Missing
formal artifacts remain `UNAVAILABLE`; do not install fake hardware or relax
safety parameters to turn them into `PASS`.

After a run, the expected process check is empty:

```bash
pgrep -af 'issue33|auto_aim_ros_e2e'
```

Rollback the reviewed feature commit with:

```bash
git revert <feature-commit-sha>
```

Do not use reset, checkout, or clean against a dirty development worktree.

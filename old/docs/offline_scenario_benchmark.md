# Offline synthetic scenario benchmark

## Scope

`auto_aim_scenario_benchmark` is a deterministic, software-only, test-only diagnostic tool. It directly constructs `TargetObservation` fixtures and explicit test-only muzzle-frame points; it does not open a camera, video, model, OpenVINO runtime, PnP solver, ROS graph, serial device, gimbal, Orin, or robot.

Its results are not hardware validation, real-trajectory validation, latency evidence, hit-rate evidence, or a competition-performance conclusion.

## Coordinate and unit boundary

Tracker association receives only synthetic `camera_xyz_m`, using the offline pipeline's OpenCV convention: `x` right, `y` down, `z` forward. It makes no calibration or external-pose claim.

Ballistic diagnostics instead receive a separately constructed point labelled `origin_assumption=synthetic_muzzle_frame`, in metres:

```text
x forward, y left, z up
```

The benchmark never derives that point from camera or gimbal data, never uses a gimbal-origin fallback, and does not add a gimbal-to-muzzle extrinsic. Relative angles are radians; timestamps are integer nanoseconds.

## Scenarios

- `static_3m`: stationary explicit 3 m muzzle-frame point;
- `spin_3m`: in-place rotation at 3 m;
- `spin_translate_3m`: synthetic rotation plus small translation;
- `crossing_permuted`: two crossing targets with seed-derived input ordering;
- `occlusion_reacquisition`: short `TempLost`, re-confirmation, timeout, then a new capture;
- `invalid_inputs`: rollback/duplicate timestamps and NaN, Inf, or invalid observations;
- `ballistic_failures`: an intentionally omitted muzzle input, unreachable
  geometry, missing/invalid bullet speed, and excessive horizon.

`all` runs these in documented order. Every scenario has fresh Tracker, Selector, Aimer, Predictor, and ballistic-diagnostic state, eliminating cross-scenario state leakage.

## Invocation and artifacts

```text
auto_aim_scenario_benchmark --scenario <name|all> --seed <uint64> --output-dir <new-path>
```

All options are required. The output directory must be new: existing directories, links, and unsafe parents fail closed before artifact creation. Success writes stable `benchmark.csv`, `summary.json`, and `summary.md`; they contain no wall-clock time, host path, or random output directory, so the same scenario and seed produce byte-identical content.

The CLI explicitly enables read-only Predictor and `OfflineBallisticDiagnostic`. Tests also compare diagnostic-enabled and diagnostic-disabled runs to prove Tracker, Selector, and `SafeOfflineAimer` semantics are unchanged.

## Safety evidence

Every record is `synthetic=true`, `test_only=true`, and `production_ready=false`. Command fields come from `SafeOfflineAimer::safe_command()`, never a diagnostic candidate:

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

`diagnostic_target_lock` is not publishable ROS or RobotCtrl state, and the safe command remains unlocked. Predictor and ballistic output are evidence only: they do not write `AimCommand`, ROS, RobotCtrl, serial, a gimbal, or firing path.

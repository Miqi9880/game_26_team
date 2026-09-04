# Release manifest audit fixtures

The `audit_test` suite creates isolated copies of these logical cases so it
never contacts ROS, a camera SDK, OpenVINO, serial, Orin, a robot, gimbal, or
firing path: valid evidence; missing and empty source; damaged JSON; unknown
schema; SHA/Git/count conflicts; contradictory safety and production claims;
output collision; deterministic rerun; and E2E missing liveness evidence.

`NOT_VERIFIED` is intentionally returned for an otherwise valid ROS E2E
report that lacks `node_liveness.alive_during_sampling`, expected exit code,
and observed exit code. It cannot contribute a release `PASS`.

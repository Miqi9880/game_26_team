# auto_aim_tools

This C++ package contains a subscription-only ROS 2 input preflight command. See
[`docs/ros_input_preflight.md`](../../docs/ros_input_preflight.md) for its
safety boundary, commands, report contract, tests, and limitations.

The package also contains SDK-independent evidence-only calibration dataset
tools. auto_aim_calibration_dataset archives a versioned offline fixture;
auto_aim_calibration_dataset_recorder subscribes only to /image_raw and
/camera_info, stores lossless PNG files, and writes a fail-closed manifest.
Both keep profile: evidence_only and production_ready: false. See
docs/calibration_dataset_evidence.md for the schemas and hash handoff.

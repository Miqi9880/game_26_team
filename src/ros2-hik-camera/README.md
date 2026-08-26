# ros2_hik_camera

A ROS 2 package for a Hikvision USB3.0 industrial camera.

> Hardware status: this repository currently contains the MVS headers only. The
> architecture-specific SDK libraries expected by `CMakeLists.txt`
> (`hikSDK/lib/amd64` or `hikSDK/lib/arm64`) are not present in the checkout.
> Obtain a licensed MVS SDK for the target machine before attempting to build or
> run this package. No Orin or real-camera test is implied by this README.

For the complete Orin, SDK, calibration, topic and safe dry-run checklist, see
[`docs/orin_real_camera_bringup.md`](../../docs/orin_real_camera_bringup.md).
The exact Image/CameraInfo wire contract and reproducible preflight sequence are
recorded in
[`docs/camera_ros_input_contract.md`](../../docs/camera_ros_input_contract.md).

## Usage

```bash
source /opt/ros/humble/setup.bash
source /path/to/game_26_dev/install/setup.bash
ros2 launch hik_camera hik_camera.launch.py \
  camera_serial:=CAMERA_SERIAL \
  frame_id:=camera_optical_frame \
  use_sensor_data_qos:=true \
  camera_info_url:=file:///absolute/path/to/verified_camera_info.yaml
```

The node allows implicit selection only when exactly one USB camera is found.
When multiple cameras are attached, set `camera_serial` to an exact serial
number; startup is rejected if the requested serial is missing or duplicated.
The default is an empty string, and no production serial number is checked in.
`camera_info_url` also defaults to empty. In that state the node publishes
matching dimensions but a deliberately uncalibrated CameraInfo, so input
preflight fails and PnP must not start. The checked-in `config/camera_info.yaml`
is an invalid, explicitly unverified format example. Before hardware
initialization, the node resolves calibration URLs and rejects package, file,
`..`, symlink, and hard-link aliases of that file. Its zero dimensions and zero
K also prevent a copied example from passing the formal CameraInfo contract.
Supply a separately verified file explicitly.
Verify `/image_raw` and `/camera_info` before starting any detector:

```bash
ros2 topic info /image_raw -v
ros2 topic info /camera_info -v
ros2 topic hz /image_raw
ros2 topic echo --once /camera_info
```

The current node publishes root topics `/image_raw` and `/camera_info` with
SensorDataQoS by default. Images are packed `rgb8`, use
`step == width * 3`, and carry `frame_id=camera_optical_frame` unless explicitly
overridden. `auto_aim_ros2` converts rgb8 to BGR before OpenCV/OpenVINO. A
production CameraInfo must be generated for the exact serial number, lens,
resolution and raw-image pipeline. A loaded formal CameraInfo whose dimensions
do not match the SDK frame causes the frame to be rejected rather than
published as a valid pair.

### Timestamp provenance

`/image_raw.header.stamp` is assigned from `this->now()` after SDK pixel
conversion and immediately before publication. It therefore represents the
camera node's ROS clock at the local publish-preparation time. It is not an
SDK exposure timestamp, hardware timestamp, IMU timestamp, or MCU timestamp.

The SDK frame timestamp is not used because this checkout does not establish a
verified mapping for it. `camera_info.header.stamp` is copied from the same
image message timestamp. This node does not read or use IMU quaternion data for
camera rotation, extrinsics, coordinate transforms, or control.

## Params

- exposure_time
- gain

`camera_serial`, `camera_name`, `camera_info_url`, `frame_id`,
`use_sensor_data_qos`, and
the white-balance parameters are also accepted. See
`config/camera_params.yaml` and the root bring-up document for units,
calibration provenance and stop conditions.

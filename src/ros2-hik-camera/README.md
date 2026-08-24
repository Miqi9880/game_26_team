# ros2_hik_camera

A ROS 2 package for a Hikvision USB3.0 industrial camera.

> Hardware status: this repository currently contains the MVS headers only. The
> architecture-specific SDK libraries expected by `CMakeLists.txt`
> (`hikSDK/lib/amd64` or `hikSDK/lib/arm64`) are not present in the checkout.
> Obtain a licensed MVS SDK for the target machine before attempting to build or
> run this package. No Orin or real-camera test is implied by this README.

For the complete Orin, SDK, calibration, topic and safe dry-run checklist, see
[`docs/orin_real_camera_bringup.md`](../../docs/orin_real_camera_bringup.md).

## Usage

```bash
source /opt/ros/humble/setup.bash
source /path/to/game_26_dev/install/setup.bash
ros2 launch hik_camera hik_camera.launch.py \
  use_sensor_data_qos:=true \
  camera_info_url:=file:///absolute/path/to/verified_camera_info.yaml
```

The node enumerates USB cameras and currently opens the first entry. Connect
only the intended camera during initial bring-up; do not rely on enumeration
order when multiple devices are attached. Verify `/image_raw` and
`/camera_info` before starting any detector:

```bash
ros2 topic info /image_raw -v
ros2 topic info /camera_info -v
ros2 topic hz /image_raw
ros2 topic echo --once /camera_info
```

The current node publishes `rgb8` images. `auto_aim_ros2` converts supported
ROS encodings to BGR before OpenCV/OpenVINO. A production `camera_info.yaml`
must be generated for the exact serial number, lens, resolution and raw-image
pipeline; the checked-in values are a format example only.

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

`camera_name`, `camera_info_url`, `use_sensor_data_qos`, and the white-balance
parameters are also accepted. See `config/camera_params.yaml` and the root
bring-up document for units, calibration provenance and stop conditions.

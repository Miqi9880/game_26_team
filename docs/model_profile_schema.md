# Detector model profile schema

`auto_aim_ros2` now has a small, versioned YAML contract for the detector
artifact.  It is separate from the PnP/calibration YAML because a model can
change tensor layout and class semantics without changing camera calibration.

The loader is:

```cpp
rm_auto_aim::detector::load_model_profile(path, options)
```

It validates the profile before any OpenVINO model is loaded.  A profile with
`profile: test_only` is rejected unless the caller explicitly sets
`ModelProfileLoadOptions::allow_test_only=true`.  No profile is inferred from
the old repository, a model cache, or a model filename.  The conversion helper
`detector_config_from_model_profile()` copies the validated tensor and
post-processing contract into `DetectorConfig`; the detector then validates the
actual IR input/output shape and element type again at OpenVINO initialization.

The detector smoke tool accepts the contract when supplied, while retaining a
deliberate unprofiled legacy smoke path for inspecting the old reference IR.
That path is diagnostic only and must not be used by PnP, offline, ROS, or
hardware-facing runs:

```bash
ros2 run auto_aim_ros2 auto_aim_detector_smoke -- \
  --model /absolute/path/to/model.xml \
  --model-profile /absolute/path/to/model_profile.yaml \
  --allow-test-profile \
  --video /absolute/path/to/video.avi
```

The PnP and full offline pipeline tools require `--model-profile` explicitly;
they do not accept an unprofiled legacy detector contract. For the checked-in
legacy fixture, opt in to both test-only boundaries:

```bash
ros2 run auto_aim_ros2 auto_aim_pnp_smoke -- \
  --model /absolute/path/to/yolov5.xml \
  --model-profile src/auto_aim_ros2/test/data/model_profile_test.yaml \
  --allow-test-profile --video /absolute/path/to/video.avi \
  --pnp-config src/auto_aim_ros2/test/data/pnp_test_config.yaml \
  --allow-test-config
```

`auto_aim_offline` uses the same `--model-profile` and
`--allow-test-profile` options. Reference-model output remains test-only and
must not be treated as a reviewed competition-model result.

## Schema version 1

The required top-level sections are:

```yaml
schema_version: 1
profile: test_only            # production only after team/model review

model:
  id: unique_model_identifier
  path: /absolute/path/to/model.xml  # production; external://... is test_only only
  source: model_provenance
  version: model_contract_version

input:
  shape: [1, 3, 640, 640]     # N,C,H,W
  layout: NCHW
  element_type: f32
  source_color_order: BGR
  model_color_order: RGB
  normalization: divide_255
  resize_mode: top_left       # or center

output:
  shape: [1, 25200, 22]       # N, rows, columns
  layout: NRC
  element_type: f32
  keypoint_count: 4
  objectness_index: 8
  color_logits_offset: 9
  color_class_count: 4
  armor_logits_offset: 13
  armor_class_count: 9

postprocess:
  objectness_threshold: 0.7
  nms_threshold: 0.3
  keypoint_order: [0, 3, 2, 1]

semantics:
  color_id_to_name: [ ... ]
  armor_class_names: [ ... ]
  class_to_armor_type:
    0: small
    1: large
```

The current decoder deliberately accepts only the explicit contract used by
the reference adapter: NCHW input with three channels, BGR→RGB preprocessing,
divide-by-255 normalization, NRC output, four keypoints, FP32 output, and the
configured tensor offsets.  Keypoint order must be a permutation of
`[0,1,2,3]`; names must be non-empty and unique; and every supported armor
class must have an explicit `small`/`large` mapping.  Unknown class semantics
are rejected instead of receiving a guessed armor size.

## Checked-in profile and production boundary

The only checked-in model profile is:

```text
src/auto_aim_ros2/test/data/model_profile_test.yaml
```

It is a `test_only` description of the legacy YOLOv5 IR used for offline smoke
tests.  Its color and armor names are deliberately labelled
`legacy_unconfirmed_*`; they are not the new competition model semantics.
There is no checked-in `production` model profile because the competition
model, output semantics, keypoint order, and versioned provenance have not yet
been confirmed.  Do not copy the legacy profile or its class mapping into a
robot configuration.

The profile does not contain model weights.  The IR/XML and BIN remain external
test inputs and are not repository artifacts.  For `profile: production`,
`model.path` must be an absolute local path to the reviewed artifact and the
runtime model path must resolve to exactly the same path; supplying a
same-shape or same-type model under another path is rejected before OpenVINO
initialization.  The current schema does not claim a content hash, so replacing
the file at that path still requires a new review/profile version and provenance
record before production use.

## Verification and bring-up order

1. Record the exact model artifact, version/hash, input/output shapes and
   element types.
2. Record preprocessing (image encoding, color order, resize/padding and
   normalization) and verify it against annotated frames.
3. Record every class/color/type semantic and the four-point order; reject
   unknown classes.
4. Add a reviewed `profile: production` YAML with a new version and provenance.
5. Run the detector smoke, PnP test, and offline ROS dry-run with
   `dry_run=true`, `serial_enabled=false`, `allow_fire=false` and
   `fire_command=0`. The PnP/offline tools must record both profile kinds and
   `effective_test_only` in their startup output.

For the ROS `offline_reference` backend, pass the reviewed profile explicitly:

```bash
ros2 run auto_aim_ros2 auto_aim_node --ros-args \
  -p backend:=offline_reference \
  -p offline_model_path:=/absolute/path/to/model.xml \
  -p offline_model_profile:=/absolute/path/to/model_profile.yaml \
  -p offline_pnp_config:=/absolute/path/to/pnp_test_or_production.yaml \
  -p allow_test_only:=true \
  -p dry_run:=true -p serial_enabled:=false -p allow_fire:=false
```

`offline_model_path` remains an explicit runtime argument.  In a `test_only`
profile it may point to an external fixture even when `model.path` is an
`external://` identifier.  In a `production` profile it must resolve to the
same absolute path as `model.path`; the detector rejects a mismatch rather than
letting an operator pair an unreviewed artifact with a reviewed semantic
contract.

The model profile is not calibration, does not define an absolute yaw/pitch
zero, and does not authorize serial, gimbal, or firing control.

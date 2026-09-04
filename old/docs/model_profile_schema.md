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
actual IR input/output shape, element type, and port layout again at OpenVINO
initialization. A profile-bound model must expose a non-empty OpenVINO input
layout matching `NCHW` and output layout matching `NRC`; layout is never
inferred from rank or dimensions, and a missing or mismatched layout is
rejected before compilation.
For `profile: production`, schema version 2 requires an explicit OpenVINO IR
manifest containing distinct XML and BIN members.  Each member has its own
absolute local path and syntactically valid SHA-256 declaration.  The runtime
validates both paths and both byte streams before calling
`Core::read_model(xml_path, bin_path)`; it never relies on OpenVINO to infer a
sibling BIN file.  The read-only qualification tool performs the same two-file
check.  No path or digest is inferred from a filename or cache.

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

## Schema version 2: production OpenVINO IR manifest

New production profiles must use schema version 2 and name both files that
form the OpenVINO IR artifact:

```yaml
schema_version: 2
profile: production

model:
  id: reviewed_model_identifier
  source: reviewed_model_provenance
  version: reviewed_model_contract_version
  format: openvino_ir
  artifacts:
    xml:
      path: /absolute/path/to/model.xml
      sha256: 64_hex_digest_for_xml
    bin:
      path: /absolute/path/to/model.bin
      sha256: 64_hex_digest_for_bin
```

The two paths must be absolute, local, regular files and must resolve to
different artifacts.  Both digests are required and independently compared
with the runtime files.  The supplied `--model` / `offline_model_path` remains
the explicit runtime XML path; for a production profile it must resolve to
the manifest's `artifacts.xml.path`.  The BIN path comes only from the reviewed
manifest and must resolve to `artifacts.bin.path`; neither member may be
substituted with an equal-shape/type file.  Only after all four comparisons
(two paths and two SHA-256 values) succeed does C++ pass both paths explicitly
to OpenVINO.

The remaining `input`, `output`, `postprocess`, and `semantics` sections are
identical to the example below.

## Schema version 1: legacy test-only compatibility

Schema version 1 is retained solely to read the checked-in external fixture.
It is never admissible as `profile: production`; its old single `model.path`
is interpreted only as a legacy XML identifier.  It does not bind weights and
must not be upgraded implicitly into a production contract.

The required top-level sections are:

```yaml
schema_version: 1
profile: test_only            # schema v1 cannot be production

model:
  id: unique_model_identifier
  path: external://.../model.xml     # legacy fixture identifier only
  source: model_provenance
  version: model_contract_version
  sha256: optional_legacy_xml_digest # never a substitute for schema-v2 BIN binding

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

## Explicit preprocessing and software-only golden evidence

The runtime builds the input tensor explicitly rather than relying on an
implicit OpenVINO color/layout conversion. For a `CV_8UC3` BGR source image of
size `(source_width, source_height)` and a reviewed NCHW canvas
`(input_width, input_height)`, it uses:

```text
scale = min(input_width / source_width, input_height / source_height)
resized_width  = max(1, int(source_width  * scale))
resized_height = max(1, int(source_height * scale))
top_left: pad_x = 0, pad_y = 0
center:   pad_x = (input_width - resized_width) / 2,
          pad_y = (input_height - resized_height) / 2
```

The canvas is black outside the resized image. Each canvas pixel `BGR(y,x)` is
converted exactly once:

```text
tensor[0, y, x] = BGR(y,x)[2] / 255  # R
tensor[1, y, x] = BGR(y,x)[1] / 255  # G
tensor[2, y, x] = BGR(y,x)[0] / 255  # B
```

Because integer raster dimensions are floored, model points use the only
inverse transform
`image_point = (model_point - [pad_x,pad_y]) / [effective_scale_x,effective_scale_y]`,
where `effective_scale_x = resized_width/source_width` and
`effective_scale_y = resized_height/source_height`. Before inversion, the
detector rejects non-finite points and points outside the actual resized
rectangle (including black padding); it also rejects points outside the
original image bounds and never clamps them into an apparent detection.

`raw_armor_detector_test` and the qualification report use fixed small-image
golden tensors and resize/padding round trips to make this mechanics
reviewable. Those values are **software preprocessing evidence only**. They
are not a model-output benchmark, an MCU raw-hex frame, a hardware golden
frame, calibration evidence, or a production-readiness claim.

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

The profile does not contain model weights.  The IR XML and BIN remain external
test inputs and are not repository artifacts.  For `profile: production`,
schema v2 binds the reviewed pair as `model.artifacts.xml` and
`model.artifacts.bin`.  Both runtime paths must match their declared absolute
paths and both SHA-256 values must match their corresponding bytes before the
detector explicitly calls `read_model(xml, bin)`.  The qualification audit also
accepts optional caller/metadata assertions for each role; these are additional
checks, never overrides for reviewed manifest values.  Replacing either XML or
BIN requires a new review/profile version and provenance record before
production use.

## Verification and bring-up order

1. Record the exact XML/BIN artifact pair, each path and SHA-256, plus the
   input/output shapes, element types, and explicit port layouts.
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

`offline_model_path` remains an explicit runtime XML argument.  In a
`test_only` legacy profile it may point to an external fixture even when
`model.path` is an `external://` identifier.  In a production schema-v2
profile it must resolve to `model.artifacts.xml.path`; the separately declared
`model.artifacts.bin.path` is also verified and passed explicitly to OpenVINO.
The detector rejects a mismatch rather than letting an operator pair an
unreviewed graph or weights file with a reviewed semantic contract.

The model profile is not calibration, does not define an absolute yaw/pitch
zero, and does not authorize serial, gimbal, or firing control.

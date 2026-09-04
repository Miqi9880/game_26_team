"""Pure, test-only evidence for the detector preprocessing contract.

This module intentionally does not load a model, invoke OpenVINO, connect to
ROS, or touch a camera.  It mirrors the *explicit* OpenCV/OpenVINO boundary in
``raw_armor_detector.cpp`` so profile reviews can produce a small, reproducible
software-only golden tensor and letterbox geometry record.  It is not a model
accuracy test, a hardware frame, or an MCU raw-hex fixture.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class PreprocessFinding:
    """A fail-closed preprocessing diagnostic independent of model loading."""

    status: str
    code: str
    message: str
    field: str | None = None

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": self.status,
            "code": self.code,
            "message": self.message,
        }
        if self.field is not None:
            result["field"] = self.field
        return result


class PreprocessContractError(ValueError):
    """One explicit reason why a preprocessing operation was rejected."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.field = field

    def finding(self) -> PreprocessFinding:
        return PreprocessFinding("FAIL", self.code, str(self), self.field)


@dataclass(frozen=True)
class PreprocessSpec:
    """The non-inferred preprocessing fields accepted by the C++ adapter."""

    input_shape: tuple[int, int, int, int]
    layout: str
    element_type: str
    source_color_order: str
    model_color_order: str
    normalization: str
    resize_mode: str

    @property
    def input_height(self) -> int:
        return self.input_shape[2]

    @property
    def input_width(self) -> int:
        return self.input_shape[3]

    def as_dict(self) -> dict[str, Any]:
        return {
            "input_shape": list(self.input_shape),
            "layout": self.layout,
            "element_type": self.element_type,
            "source_color_order": self.source_color_order,
            "model_color_order": self.model_color_order,
            "normalization": self.normalization,
            "resize_mode": self.resize_mode,
            "padding_rule": self.resize_mode,
        }


@dataclass(frozen=True)
class LetterboxGeometry:
    """The exact scale/padding values needed to invert model pixel points.

    ``scale`` is the nominal float32 scale used to select the resized extent.
    Since OpenCV receives integer resized dimensions, inverse coordinates must
    use the effective per-axis scales derived from those dimensions instead.
    """

    source_width: int
    source_height: int
    canvas_width: int
    canvas_height: int
    resized_width: int
    resized_height: int
    scale: float
    effective_scale_x: float
    effective_scale_y: float
    pad_x: int
    pad_y: int
    resize_mode: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "source_size": [self.source_width, self.source_height],
            "canvas_size": [self.canvas_width, self.canvas_height],
            "resized_size": [self.resized_width, self.resized_height],
            "nominal_scale": self.scale,
            "effective_scale_x": self.effective_scale_x,
            "effective_scale_y": self.effective_scale_y,
            "pad_x": self.pad_x,
            "pad_y": self.pad_y,
            "resize_mode": self.resize_mode,
        }


@dataclass(frozen=True)
class PreprocessedTensor:
    """A uint8 letterbox canvas plus the explicit model f32 NCHW tensor."""

    canvas_bgr: Any
    tensor_nchw: Any
    geometry: LetterboxGeometry


def _backends() -> tuple[Any, Any]:
    """Load the two required numerical backends or reject rather than guess."""

    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on minimal host image.
        raise PreprocessContractError(
            "preprocess_runtime_unavailable",
            f"numpy/OpenCV preprocessing runtime is unavailable: {type(exc).__name__}",
        ) from exc
    return np, cv2


def _positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def spec_from_input_contract(input_node: Mapping[str, Any]) -> PreprocessSpec:
    """Parse the profile input section without adding implicit conventions."""

    shape_value = input_node.get("shape")
    if not isinstance(shape_value, Sequence) or isinstance(shape_value, (str, bytes)):
        raise PreprocessContractError("preprocess_input_shape", "input.shape must be a four-item NCHW sequence", field="input.shape")
    shape = list(shape_value)
    if len(shape) != 4 or not all(_positive_int(item) for item in shape) or shape[:2] != [1, 3]:
        raise PreprocessContractError(
            "preprocess_input_shape",
            "input.shape must be exactly [1,3,height,width] with positive dimensions",
            field="input.shape",
        )

    expected = {
        "layout": "NCHW",
        "element_type": "f32",
        "source_color_order": "BGR",
        "model_color_order": "RGB",
        "normalization": "divide_255",
    }
    values: dict[str, str] = {}
    for field, required in expected.items():
        raw = input_node.get(field)
        if not isinstance(raw, str) or raw != required:
            raise PreprocessContractError(
                f"preprocess_{field}",
                f"input.{field} must be exactly {required}",
                field=f"input.{field}",
            )
        values[field] = raw
    resize_mode = input_node.get("resize_mode")
    if resize_mode not in {"top_left", "center"}:
        raise PreprocessContractError(
            "preprocess_resize_mode",
            "input.resize_mode must be top_left or center; implicit padding is not accepted",
            field="input.resize_mode",
        )
    return PreprocessSpec(
        tuple(int(item) for item in shape),
        values["layout"],
        values["element_type"],
        values["source_color_order"],
        values["model_color_order"],
        values["normalization"],
        str(resize_mode),
    )


def letterbox_geometry(
    source_width: int,
    source_height: int,
    canvas_width: int,
    canvas_height: int,
    resize_mode: str,
) -> LetterboxGeometry:
    """Mirror C++ ``make_letterbox`` geometry, including float32 rounding."""

    if not all(_positive_int(value) for value in (source_width, source_height, canvas_width, canvas_height)):
        raise PreprocessContractError("image_dimensions", "source and canvas dimensions must be positive integers")
    if resize_mode not in {"top_left", "center"}:
        raise PreprocessContractError("preprocess_resize_mode", "resize_mode must be top_left or center")
    np, _ = _backends()
    # C++ calculates these operands as float, so retain float32 before
    # truncating resized dimensions exactly as ``static_cast<int>`` does.
    scale = min(
        np.float32(canvas_width) / np.float32(source_width),
        np.float32(canvas_height) / np.float32(source_height),
    )
    scale_value = float(scale)
    if not math.isfinite(scale_value) or scale_value <= 0.0:
        raise PreprocessContractError("resize_scale", "letterbox scale must be finite and positive")
    resized_width = max(1, int(np.float32(source_width) * scale))
    resized_height = max(1, int(np.float32(source_height) * scale))
    effective_scale_x = float(resized_width) / float(source_width)
    effective_scale_y = float(resized_height) / float(source_height)
    available_width = canvas_width - resized_width
    available_height = canvas_height - resized_height
    pad_x = available_width // 2 if resize_mode == "center" else 0
    pad_y = available_height // 2 if resize_mode == "center" else 0
    if (
        resized_width <= 0
        or resized_height <= 0
        or pad_x < 0
        or pad_y < 0
        or pad_x + resized_width > canvas_width
        or pad_y + resized_height > canvas_height
        or not math.isfinite(effective_scale_x)
        or not math.isfinite(effective_scale_y)
        or effective_scale_x <= 0.0
        or effective_scale_y <= 0.0
    ):
        raise PreprocessContractError("letterbox_geometry", "resize/padding does not fit the declared canvas")
    return LetterboxGeometry(
        source_width,
        source_height,
        canvas_width,
        canvas_height,
        resized_width,
        resized_height,
        scale_value,
        effective_scale_x,
        effective_scale_y,
        pad_x,
        pad_y,
        resize_mode,
    )


def _validate_bgr_image(image: Any) -> tuple[Any, int, int]:
    np, _ = _backends()
    if not isinstance(image, np.ndarray):
        raise PreprocessContractError("image_type", "input image must be a numpy ndarray")
    if image.ndim != 3 or image.shape[2] != 3:
        raise PreprocessContractError("image_shape", "input image must have HxWx3 channels")
    if image.shape[0] <= 0 or image.shape[1] <= 0:
        raise PreprocessContractError("image_empty", "input image must have positive width and height")
    if not np.issubdtype(image.dtype, np.number):
        raise PreprocessContractError("image_type", "input image dtype must be numeric")
    if not np.isfinite(image).all():
        raise PreprocessContractError("image_nonfinite", "input image contains NaN or Inf")
    if image.dtype != np.uint8:
        raise PreprocessContractError("image_type", "input image must be uint8 BGR pixels")
    return image, int(image.shape[1]), int(image.shape[0])


def letterbox_bgr(image: Any, spec: PreprocessSpec) -> tuple[Any, LetterboxGeometry]:
    """Return the explicit zero-padded BGR canvas; no color conversion occurs."""

    np, cv2 = _backends()
    image, source_width, source_height = _validate_bgr_image(image)
    geometry = letterbox_geometry(
        source_width,
        source_height,
        spec.input_width,
        spec.input_height,
        spec.resize_mode,
    )
    try:
        resized = cv2.resize(
            image,
            (geometry.resized_width, geometry.resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
    except Exception as exc:
        raise PreprocessContractError("resize_failed", f"explicit OpenCV INTER_LINEAR resize failed: {type(exc).__name__}") from exc
    if not isinstance(resized, np.ndarray) or resized.shape != (geometry.resized_height, geometry.resized_width, 3):
        raise PreprocessContractError("resize_failed", "explicit resize produced an unexpected shape")
    canvas = np.zeros((spec.input_height, spec.input_width, 3), dtype=np.uint8)
    canvas[
        geometry.pad_y:geometry.pad_y + geometry.resized_height,
        geometry.pad_x:geometry.pad_x + geometry.resized_width,
    ] = resized
    return canvas, geometry


def preprocess_bgr_to_nchw(image: Any, spec: PreprocessSpec) -> PreprocessedTensor:
    """Apply only the reviewed BGR→RGB, divide-255, f32 NCHW transform."""

    np, _ = _backends()
    canvas, geometry = letterbox_bgr(image, spec)
    # These operations intentionally remain explicit.  Do not replace them
    # with a generic blob helper, implicit crop, or inferred color order.
    rgb = canvas[:, :, ::-1]
    tensor = np.transpose(rgb.astype(np.float32) / np.float32(255.0), (2, 0, 1))[np.newaxis, ...]
    if tuple(int(item) for item in tensor.shape) != spec.input_shape:
        raise PreprocessContractError("tensor_shape", "preprocessed tensor does not match declared NCHW input shape")
    if tensor.dtype != np.float32 or not np.isfinite(tensor).all():
        raise PreprocessContractError("tensor_nonfinite", "preprocessed tensor must be finite f32")
    return PreprocessedTensor(canvas, tensor, geometry)


def image_point_to_model(point: Sequence[Any], geometry: LetterboxGeometry) -> tuple[float, float]:
    """Forward-map one in-image point using the reviewed scale/padding rule."""

    if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) != 2:
        raise PreprocessContractError("point_shape", "point must contain exactly two coordinates")
    try:
        x, y = float(point[0]), float(point[1])
    except (TypeError, ValueError) as exc:
        raise PreprocessContractError("point_type", "point coordinates must be numeric") from exc
    if not all(math.isfinite(value) for value in (x, y, geometry.effective_scale_x, geometry.effective_scale_y)):
        raise PreprocessContractError("point_nonfinite", "point and effective resize scales must be finite")
    if geometry.effective_scale_x <= 0.0 or geometry.effective_scale_y <= 0.0:
        raise PreprocessContractError("resize_scale", "effective resize scales must be finite and positive")
    if not (0.0 <= x < geometry.source_width and 0.0 <= y < geometry.source_height):
        raise PreprocessContractError("point_out_of_image", "source keypoint is outside image bounds")
    model_x = x * geometry.effective_scale_x + float(geometry.pad_x)
    model_y = y * geometry.effective_scale_y + float(geometry.pad_y)
    if not (
        float(geometry.pad_x) <= model_x < float(geometry.pad_x + geometry.resized_width)
        and float(geometry.pad_y) <= model_y < float(geometry.pad_y + geometry.resized_height)
    ):
        raise PreprocessContractError("letterbox_geometry", "forward point does not fit the actual resized rectangle")
    return model_x, model_y


def model_point_to_image(point: Sequence[Any], geometry: LetterboxGeometry) -> tuple[float, float]:
    """Invert one model point after rejecting all letterbox-padding points."""

    if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) != 2:
        raise PreprocessContractError("point_shape", "point must contain exactly two coordinates")
    try:
        x, y = float(point[0]), float(point[1])
    except (TypeError, ValueError) as exc:
        raise PreprocessContractError("point_type", "point coordinates must be numeric") from exc
    if not all(math.isfinite(value) for value in (x, y, geometry.effective_scale_x, geometry.effective_scale_y)):
        raise PreprocessContractError("point_nonfinite", "model point and effective resize scales must be finite")
    if geometry.effective_scale_x <= 0.0 or geometry.effective_scale_y <= 0.0:
        raise PreprocessContractError("resize_scale", "effective resize scales must be finite and positive")
    # Reject before inverse scaling: a point in top/left/right/bottom padding
    # can otherwise land on a numerically plausible source coordinate.
    if not (
        float(geometry.pad_x) <= x < float(geometry.pad_x + geometry.resized_width)
        and float(geometry.pad_y) <= y < float(geometry.pad_y + geometry.resized_height)
    ):
        raise PreprocessContractError(
            "model_point_outside_resized_rect",
            "model keypoint is outside the actual resized rectangle (letterbox padding is invalid)",
        )
    image_x = (x - float(geometry.pad_x)) / geometry.effective_scale_x
    image_y = (y - float(geometry.pad_y)) / geometry.effective_scale_y
    if not all(math.isfinite(value) for value in (image_x, image_y)):
        raise PreprocessContractError("point_nonfinite", "inverse point is non-finite")
    if not (0.0 <= image_x < geometry.source_width and 0.0 <= image_y < geometry.source_height):
        raise PreprocessContractError(
            "point_out_of_image",
            "model keypoint maps outside original image bounds; rejecting fail-closed",
        )
    return image_x, image_y


def model_keypoints_to_image(points: Sequence[Sequence[Any]], geometry: LetterboxGeometry) -> list[tuple[float, float]]:
    """Invert exactly four finite, in-image armor keypoints or reject all."""

    if not isinstance(points, Sequence) or isinstance(points, (str, bytes)) or len(points) != 4:
        raise PreprocessContractError("keypoint_count", "exactly four model keypoints are required")
    return [model_point_to_image(point, geometry) for point in points]


def _round_trip_record(geometry: LetterboxGeometry, point: tuple[float, float]) -> dict[str, Any]:
    model_point = image_point_to_model(point, geometry)
    restored = model_point_to_image(model_point, geometry)
    return {
        "source_point": [point[0], point[1]],
        "model_point": [model_point[0], model_point[1]],
        "restored_point": [restored[0], restored[1]],
        "round_trip_error": max(abs(point[0] - restored[0]), abs(point[1] - restored[1])),
    }


def software_preprocessing_evidence(input_node: Mapping[str, Any]) -> dict[str, Any]:
    """Produce compact deterministic software evidence for a profile's input contract.

    The canonical 2x2 tensor makes channel ordering and normalization visible
    without publishing a large model-sized tensor.  A separate 2x1→4x4
    geometry fixture makes scale and both padding modes auditable, while a
    non-integer fixture proves that inverse mapping uses actual resized axes.
    Neither fixture is a model output, an MCU frame, or a hardware validation
    claim.
    """

    try:
        spec = spec_from_input_contract(input_node)
        np, _ = _backends()
        fixture = np.asarray(
            [
                [[1, 2, 3], [4, 5, 6]],
                [[7, 8, 9], [10, 11, 12]],
            ],
            dtype=np.uint8,
        )
        # Tiny exact tensor fixture: no resize means every asserted value is
        # independent of interpolation implementation details.
        mini_spec = PreprocessSpec(
            (1, 3, 2, 2),
            spec.layout,
            spec.element_type,
            spec.source_color_order,
            spec.model_color_order,
            spec.normalization,
            "top_left",
        )
        mini = preprocess_bgr_to_nchw(fixture, mini_spec)
        top_left = letterbox_geometry(2, 1, 4, 4, "top_left")
        center = letterbox_geometry(2, 1, 4, 4, "center")
        top_left_round_trip = _round_trip_record(top_left, (0.5, 0.25))
        center_round_trip = _round_trip_record(center, (0.5, 0.25))
        try:
            # x=0,y=0 lies inside center's top padding (pad_y=1) and must
            # never be accepted as an original-image keypoint.
            model_point_to_image((0.0, 0.0), center)
        except PreprocessContractError as exc:
            out_of_bounds = {"rejected": True, "code": exc.code}
        else:  # pragma: no cover - guard against a future unsafe relaxation.
            out_of_bounds = {"rejected": False, "code": "unsafe_accept"}

        # ``333 * 0.64`` floors to 213 rows.  The effective y scale is
        # therefore 213/333, not the nominal 0.64 used to choose dimensions.
        noninteger_top_left = letterbox_geometry(1000, 333, 640, 640, "top_left")
        noninteger_center = letterbox_geometry(1000, 333, 640, 640, "center")
        noninteger_top_left_round_trip = _round_trip_record(noninteger_top_left, (500.0, 100.0))
        noninteger_center_round_trip = _round_trip_record(noninteger_center, (500.0, 100.0))
        try:
            # The top-left point is exactly one row below the actual 213-row
            # resized rectangle.  It must be rejected before inversion.
            model_point_to_image((0.0, float(noninteger_top_left.resized_height)), noninteger_top_left)
        except PreprocessContractError as exc:
            noninteger_out_of_bounds = {"rejected": True, "code": exc.code}
        else:  # pragma: no cover - guard against a future unsafe relaxation.
            noninteger_out_of_bounds = {"rejected": False, "code": "unsafe_accept"}

        declared_geometry = letterbox_geometry(2, 2, spec.input_width, spec.input_height, spec.resize_mode)
        declared = preprocess_bgr_to_nchw(fixture, spec)
        mini_tensor = mini.tensor_nchw
        return {
            "status": "PASS" if out_of_bounds["rejected"] and noninteger_out_of_bounds["rejected"] else "FAIL",
            "kind": "software_preprocessing_golden",
            "software_evidence_only": True,
            "hardware_validation": False,
            "mcu_raw_hex_fixture": False,
            "model_inference_executed": False,
            "contract": spec.as_dict(),
            "fixed_bgr_fixture": {
                "shape_hwc": [2, 2, 3],
                "pixels_bgr_u8": fixture.tolist(),
                "expected_tensor_shape_nchw": [1, 3, 2, 2],
                "expected_tensor_f32_nchw": mini_tensor.tolist(),
                "tensor_sha256": hashlib.sha256(mini_tensor.tobytes()).hexdigest(),
            },
            "declared_shape_summary": {
                "tensor_shape_nchw": [int(item) for item in declared.tensor_nchw.shape],
                "tensor_element_type": str(declared.tensor_nchw.dtype),
                "letterbox": declared_geometry.as_dict(),
                "sample_rgb_f32_at_0_0": [float(declared.tensor_nchw[0, channel, 0, 0]) for channel in range(3)],
                "tensor_sha256": hashlib.sha256(declared.tensor_nchw.tobytes()).hexdigest(),
            },
            "letterbox_geometry_fixture": {
                "top_left": top_left.as_dict(),
                "center": center.as_dict(),
                "top_left_round_trip": top_left_round_trip,
                "center_round_trip": center_round_trip,
                "padding_keypoint_out_of_bounds": out_of_bounds,
                "noninteger_scale_regression": {
                    "top_left": noninteger_top_left.as_dict(),
                    "center": noninteger_center.as_dict(),
                    "top_left_round_trip": noninteger_top_left_round_trip,
                    "center_round_trip": noninteger_center_round_trip,
                    "outside_actual_resized_rect": noninteger_out_of_bounds,
                },
            },
            "findings": ([] if out_of_bounds["rejected"] and noninteger_out_of_bounds["rejected"] else [
                PreprocessFinding(
                    "FAIL",
                    "model_point_outside_resized_rect",
                    "a padding/outside-resize keypoint was accepted",
                ).as_dict()
            ]),
        }
    except PreprocessContractError as exc:
        return {
            "status": "FAIL",
            "kind": "software_preprocessing_golden",
            "software_evidence_only": True,
            "hardware_validation": False,
            "mcu_raw_hex_fixture": False,
            "model_inference_executed": False,
            "contract": {},
            "findings": [exc.finding().as_dict()],
        }


__all__ = [
    "LetterboxGeometry",
    "PreprocessContractError",
    "PreprocessFinding",
    "PreprocessSpec",
    "PreprocessedTensor",
    "image_point_to_model",
    "letterbox_bgr",
    "letterbox_geometry",
    "model_keypoints_to_image",
    "model_point_to_image",
    "preprocess_bgr_to_nchw",
    "software_preprocessing_evidence",
    "spec_from_input_contract",
]

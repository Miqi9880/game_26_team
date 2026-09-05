"""Golden and fail-closed tests for the pure preprocessing evidence module."""

from __future__ import annotations

import unittest

import numpy as np

from tools.auto_aim_qualification.preprocess_contract import (
    PreprocessContractError,
    PreprocessSpec,
    image_point_to_model,
    letterbox_geometry,
    model_keypoints_to_image,
    model_point_to_image,
    preprocess_bgr_to_nchw,
    software_preprocessing_evidence,
)


class PreprocessContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = PreprocessSpec(
            (1, 3, 2, 2), "NCHW", "f32", "BGR", "RGB", "divide_255", "top_left"
        )
        self.fixture = np.asarray(
            [
                [[1, 2, 3], [4, 5, 6]],
                [[7, 8, 9], [10, 11, 12]],
            ],
            dtype=np.uint8,
        )

    def test_fixed_bgr_fixture_has_explicit_rgb_divide_255_nchw_tensor(self):
        result = preprocess_bgr_to_nchw(self.fixture, self.spec)
        expected = np.asarray(
            [[[
                [3, 6], [9, 12],
            ], [
                [2, 5], [8, 11],
            ], [
                [1, 4], [7, 10],
            ]]],
            dtype=np.float32,
        ) / np.float32(255.0)
        self.assertEqual(tuple(result.tensor_nchw.shape), (1, 3, 2, 2))
        self.assertEqual(result.tensor_nchw.dtype, np.float32)
        np.testing.assert_array_equal(result.tensor_nchw, expected)
        self.assertEqual((result.geometry.scale, result.geometry.pad_x, result.geometry.pad_y), (1.0, 0, 0))

    def test_top_left_center_scale_inverse_and_padding_point_rejection(self):
        top_left = letterbox_geometry(2, 1, 4, 4, "top_left")
        center = letterbox_geometry(2, 1, 4, 4, "center")
        self.assertEqual((top_left.scale, top_left.resized_width, top_left.resized_height, top_left.pad_x, top_left.pad_y), (2.0, 4, 2, 0, 0))
        self.assertEqual((center.scale, center.resized_width, center.resized_height, center.pad_x, center.pad_y), (2.0, 4, 2, 0, 1))
        for geometry, expected in ((top_left, (1.0, 0.5)), (center, (1.0, 1.5))):
            model = image_point_to_model((0.5, 0.25), geometry)
            self.assertEqual(model, expected)
            restored = model_point_to_image(model, geometry)
            self.assertAlmostEqual(restored[0], 0.5)
            self.assertAlmostEqual(restored[1], 0.25)
        with self.assertRaisesRegex(PreprocessContractError, "outside the actual resized rectangle"):
            model_keypoints_to_image([(0.0, 0.0)] * 4, center)

    def test_noninteger_resize_uses_effective_axis_scales_and_rejects_padding(self):
        # The nominal scale is 0.64, but the floored height is 213 rather than
        # 213.12.  Inverse mapping must use 213 / 333 to match actual pixels.
        cases = (
            ("top_left", 0),
            ("center", (640 - 213) // 2),
        )
        for resize_mode, expected_pad_y in cases:
            with self.subTest(resize_mode=resize_mode):
                geometry = letterbox_geometry(1000, 333, 640, 640, resize_mode)
                self.assertEqual((geometry.resized_width, geometry.resized_height), (640, 213))
                self.assertAlmostEqual(geometry.scale, 0.64, places=6)
                self.assertAlmostEqual(geometry.effective_scale_x, 640.0 / 1000.0, places=12)
                self.assertAlmostEqual(geometry.effective_scale_y, 213.0 / 333.0, places=12)
                self.assertNotAlmostEqual(geometry.effective_scale_y, geometry.scale, places=4)
                self.assertEqual(geometry.pad_y, expected_pad_y)
                model = image_point_to_model((500.0, 100.0), geometry)
                self.assertAlmostEqual(model[0], 320.0, places=12)
                self.assertAlmostEqual(model[1], expected_pad_y + 100.0 * 213.0 / 333.0, places=12)
                restored = model_point_to_image(model, geometry)
                self.assertAlmostEqual(restored[0], 500.0, places=9)
                self.assertAlmostEqual(restored[1], 100.0, places=9)
                outside_y = float(geometry.resized_height) if resize_mode == "top_left" else float(geometry.pad_y) - 0.1
                with self.assertRaises(PreprocessContractError) as raised:
                    model_point_to_image((0.0, outside_y), geometry)
                self.assertEqual(raised.exception.code, "model_point_outside_resized_rect")

    def test_empty_wrong_type_and_nonfinite_images_fail_closed(self):
        invalid_cases = (
            (np.zeros((0, 2, 3), dtype=np.uint8), "image_empty"),
            (np.zeros((2, 2, 2), dtype=np.uint8), "image_shape"),
            (np.zeros((2, 2, 3), dtype=np.float32), "image_type"),
            (np.full((2, 2, 3), np.nan, dtype=np.float32), "image_nonfinite"),
            (np.full((2, 2, 3), np.inf, dtype=np.float32), "image_nonfinite"),
        )
        for image, code in invalid_cases:
            with self.subTest(code=code):
                with self.assertRaises(PreprocessContractError) as raised:
                    preprocess_bgr_to_nchw(image, self.spec)
                self.assertEqual(raised.exception.code, code)
        geometry = letterbox_geometry(2, 1, 4, 4, "center")
        for point in ((float("nan"), 0.0), (float("inf"), 0.0), (4.0, 4.0)):
            with self.subTest(point=point):
                with self.assertRaises(PreprocessContractError):
                    model_point_to_image(point, geometry)

    def test_evidence_is_explicitly_software_only_and_contains_golden_tensor(self):
        evidence = software_preprocessing_evidence({
            "shape": [1, 3, 640, 640],
            "layout": "NCHW",
            "element_type": "f32",
            "source_color_order": "BGR",
            "model_color_order": "RGB",
            "normalization": "divide_255",
            "resize_mode": "top_left",
        })
        self.assertEqual(evidence["status"], "PASS")
        self.assertTrue(evidence["software_evidence_only"])
        self.assertFalse(evidence["hardware_validation"])
        self.assertFalse(evidence["mcu_raw_hex_fixture"])
        self.assertEqual(evidence["fixed_bgr_fixture"]["expected_tensor_shape_nchw"], [1, 3, 2, 2])
        geometry = evidence["letterbox_geometry_fixture"]
        self.assertEqual(
            geometry["padding_keypoint_out_of_bounds"],
            {"rejected": True, "code": "model_point_outside_resized_rect"},
        )
        noninteger = geometry["noninteger_scale_regression"]
        self.assertEqual(noninteger["top_left"]["resized_size"], [640, 213])
        self.assertNotEqual(
            noninteger["top_left"]["nominal_scale"],
            noninteger["top_left"]["effective_scale_y"],
        )
        self.assertEqual(
            noninteger["outside_actual_resized_rect"],
            {"rejected": True, "code": "model_point_outside_resized_rect"},
        )


if __name__ == "__main__":
    unittest.main()

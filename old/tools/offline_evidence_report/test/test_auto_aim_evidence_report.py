import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "offline_evidence_report" / "auto_aim_evidence_report.py"
FIXTURES = ROOT / "tools" / "offline_evidence_report" / "fixtures"
sys.path.insert(0, str(ROOT))

from tools.offline_evidence_report.auto_aim_evidence_report import (  # noqa: E402
    BALLISTIC_COLUMNS,
    analyze_csv,
    build_report,
    markdown_report,
    write_reports,
)

def render_markdown(source):
    return markdown_report(build_report(source) if not isinstance(source, dict) else source)

def report_for(path):
    return build_report(analyze_csv(path))


HEADER = [
    "frame", "stamp_ns", "tracking_state", "selected", "target_lock", "fire_command", "test_only",
    "detection_count", "valid_pnp_count", "pnp_valid", "pnp_failure", "reprojection_error_px", "track_id",
]
# This mirrors the actual auto_aim_offline header: prediction flags precede
# the append-only ballistic suffix, while generic command safety fields are
# part of that suffix's final safety block.
ACTUAL_PREDICTION_COLUMNS = ["prediction_valid", "prediction_reason", "prediction_horizon_ms",
                             "prediction_source_stamp_ns", "prediction_stamp_ns",
                             "predicted_relative_yaw_rad", "predicted_relative_pitch_rad",
                             "predicted_relative_yaw_degree", "predicted_relative_pitch_degree",
                             "prediction_test_only", "prediction_production_ready"]
BALLISTIC_HEADER = HEADER + ACTUAL_PREDICTION_COLUMNS + list(BALLISTIC_COLUMNS)


def write_csv(path, rows, header=HEADER):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            values = list(row)
            if len(values) < len(header):
                values.extend([""] * (len(header) - len(values)))
            writer.writerow(values)


def ballistic_row(**overrides):
    values = {
        "frame": 0,
        "stamp_ns": 10,
        "tracking_state": "tracking",
        "selected": 1,
        "target_lock": 49,
        "fire_command": 0,
        "test_only": 1,
        "detection_count": 1,
        "valid_pnp_count": 1,
        "pnp_valid": 1,
        "pnp_failure": "",
        "reprojection_error_px": 0.1,
        "track_id": 7,
        "prediction_valid": 0,
        "prediction_reason": "disabled",
        "prediction_test_only": 1,
        "prediction_production_ready": 0,
        "ballistic_enabled": 1,
        "ballistic_valid": 1,
        "ballistic_reason": "none",
        "ballistic_track_id": 7,
        "ballistic_source_stamp_ns": 10,
        "ballistic_target_x_m": 3.0,
        "ballistic_target_y_m": 0.0,
        "ballistic_target_z_m": 0.0,
        "ballistic_horizontal_distance_m": 3.0,
        "ballistic_geometric_yaw_rad": 0.0,
        "ballistic_geometric_pitch_rad": 0.0,
        "ballistic_yaw_rad": 0.0,
        "ballistic_pitch_rad": 0.04,
        "ballistic_gravity_pitch_correction_rad": 0.04,
        "ballistic_flight_time_s": 0.15,
        "ballistic_flight_time_ns": 150000000,
        "ballistic_system_latency_ns": 10000000,
        "ballistic_recommended_prediction_horizon_ns": 160000000,
        "ballistic_bullet_speed_mps": 20.0,
        "ballistic_gravity_mps2": 9.81,
        "ballistic_origin_assumption": "test_only_gimbal_origin",
        "ballistic_test_only": 1,
        "ballistic_production_ready": 0,
        "ballistic_control_applied": 0,
        "serial_enabled": 0,
        "dry_run": 1,
        "allow_fire": 0,
        "yaw_vel_rad_s": 0.0,
        "pitch_vel_rad_s": 0.0,
        "yaw_acc_rad_s2": 0.0,
        "pitch_acc_rad_s2": 0.0,
    }
    values.update(overrides)
    return [values.get(column, "") for column in BALLISTIC_HEADER]


class EvidenceReportTests(unittest.TestCase):
    def test_valid_ballistic_suffix_is_test_only_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ballistic.csv"
            write_csv(path, [ballistic_row()], BALLISTIC_HEADER)
            report = report_for(path)
        self.assertEqual(report["report_status"], "PASS")
        self.assertTrue(report["ballistic_diagnostics"]["present"])
        self.assertEqual(report["ballistic_diagnostics"]["valid_distribution"], {"True": 1})
        self.assertEqual(
            report["ballistic_diagnostics"]["origin_assumption_distribution"],
            {"test_only_gimbal_origin": 1},
        )
        self.assertIn("Ballistic diagnostic (test-only)", markdown_report(report))
        self.assertEqual(report["ballistic_diagnostics"]["gravity_pitch_correction_rad"]["count"], 1)

    def test_legacy_prediction_suffix_does_not_imply_ballistic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.csv"
            header = HEADER + ACTUAL_PREDICTION_COLUMNS
            values = [0, 10, "lost", 0, 50, 0, 1, 0, 0, 0, "", "", 0]
            write_csv(path, [values], header)
            report = report_for(path)
        self.assertEqual(report["report_status"], "PASS")
        self.assertFalse(report["ballistic_diagnostics"]["present"])

    def test_prediction_test_only_false_is_a_safety_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy_unsafe.csv"
            header = HEADER + ACTUAL_PREDICTION_COLUMNS
            values = [0, 10, "lost", 0, 50, 0, 1, 0, 0, 0, "", "", 0]
            values.extend([""] * (len(header) - len(values)))
            values[header.index("prediction_test_only")] = 0
            write_csv(path, [values], header)
            report = report_for(path)
        self.assertEqual(report["report_status"], "FAIL")
        self.assertIn("test_only_false", report["safety"]["anomalies"])

    def test_actual_header_shape_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actual.csv"
            write_csv(path, [ballistic_row()], BALLISTIC_HEADER)
            report = report_for(path)
        self.assertEqual(report["report_status"], "PASS")
        self.assertEqual(report["input"]["columns"][-len(BALLISTIC_COLUMNS):], list(BALLISTIC_COLUMNS))

    def test_ballistic_state_and_units_fail_closed(self):
        cases = (
            {"ballistic_enabled": 0, "ballistic_valid": 1},
            {"ballistic_origin_assumption": "evil"},
            {"ballistic_flight_time_s": -0.1},
            {"ballistic_bullet_speed_mps": 0.0},
            {"ballistic_flight_time_s": 0.15, "ballistic_flight_time_ns": 1},
        )
        for overrides in cases:
            with self.subTest(overrides=overrides), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "invalid.csv"
                write_csv(path, [ballistic_row(**overrides)], BALLISTIC_HEADER)
                report = report_for(path)
            self.assertEqual(report["report_status"], "FAIL")

    def test_incomplete_ballistic_suffix_and_nonfinite_values_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            incomplete = root / "incomplete.csv"
            write_csv(incomplete, [[0, 10, "lost", 0, 50, 0, 1]], HEADER + ["ballistic_enabled"])
            incomplete_report = report_for(incomplete)
            self.assertEqual(incomplete_report["report_status"], "FAIL")
            self.assertTrue(any(item["code"] == "incomplete_ballistic_columns" for item in incomplete_report["errors"]))

            nonfinite = root / "nonfinite.csv"
            write_csv(nonfinite, [ballistic_row(ballistic_flight_time_s="nan")], BALLISTIC_HEADER)
            nonfinite_report = report_for(nonfinite)
            self.assertEqual(nonfinite_report["report_status"], "FAIL")
            self.assertTrue(any(item["code"] == "invalid_finite_float" for item in nonfinite_report["errors"]))

    def test_ballistic_suffix_rejects_production_or_control_safety_violations(self):
        unsafe_values = {
            "ballistic_test_only": 0,
            "ballistic_production_ready": 1,
            "ballistic_control_applied": 1,
            "serial_enabled": 1,
            "dry_run": 0,
            "allow_fire": 1,
            "yaw_vel_rad_s": 0.01,
            "pitch_acc_rad_s2": -0.01,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe_ballistic.csv"
            write_csv(path, [ballistic_row(**unsafe_values)], BALLISTIC_HEADER)
            report = report_for(path)
        self.assertEqual(report["report_status"], "FAIL")
        anomalies = set(report["safety"]["anomalies"])
        self.assertIn("ballistic_test_only_false", anomalies)
        self.assertIn("ballistic_production_ready_true", anomalies)
        self.assertIn("ballistic_control_applied", anomalies)
        self.assertIn("serial_enabled", anomalies)
        self.assertIn("dry_run_false", anomalies)
        self.assertIn("allow_fire_true", anomalies)
    def test_normal_csv_passes(self):
        report = report_for(FIXTURES / "normal.csv")
        self.assertEqual(report["report_status"], "PASS")
        self.assertEqual(report["coverage"]["unique_frame_count"], 3)
        self.assertEqual(report["tracker_target"]["target_lock_49_frames"], 2)

    def test_missing_required_column_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.csv"
            write_csv(path, [[0, 1, "lost", 0, 50, 0]], [column for column in HEADER if column != "test_only"])
            report = report_for(path)
        self.assertEqual(report["report_status"], "FAIL")
        self.assertTrue(any("required CSV columns are missing" in item["message"] for item in report["errors"]))

    def test_empty_csv_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.csv"
            path.write_text("", encoding="utf-8")
            report = report_for(path)
        self.assertEqual(report["report_status"], "FAIL")

    def test_row_length_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.csv"
            path.write_text(",".join(HEADER) + "\n0,1,lost\n", encoding="utf-8")
            report = report_for(path)
        self.assertTrue(any(item["code"] == "row_length_mismatch" for item in report["errors"]))

    def test_nan_inf_are_rejected(self):
        report = report_for(FIXTURES / "anomaly.csv")
        errors = " ".join(item["message"] for item in report["errors"])
        self.assertIn("invalid finite float", errors)

    def test_timestamp_rollback_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rollback.csv"
            write_csv(path, [[0, 10, "lost", 0, 50, 0, 1], [1, 9, "lost", 0, 50, 0, 1]])
            report = report_for(path)
        self.assertTrue(any(item["code"] == "timestamp_rollback" for item in report["errors"]))

    def test_frame_gap_is_counted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gap.csv"
            write_csv(path, [[0, 10, "lost", 0, 50, 0, 1], [2, 20, "lost", 0, 50, 0, 1]])
            report = report_for(path)
        self.assertEqual(report["coverage"]["missing_frames"], [1])
        self.assertEqual(report["report_status"], "WARN")

    def test_duplicate_frame_rows_are_aggregated_once(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.csv"
            write_csv(path, [
                [0, 10, "tracking", 1, 49, 0, 1, 2, 2, 1, "", 0.1, 1],
                [0, 10, "tracking", 0, 49, 0, 1, 2, 2, 1, "", 0.2, 1],
                [1, 20, "tracking", 1, 49, 0, 1, 1, 1, 1, "", 0.1, 1],
            ])
            report = report_for(path)
        self.assertEqual(report["tracker_target"]["target_lock_49_frames"], 2)
        self.assertEqual(report["coverage"]["duplicate_frame_rows"], 1)

    def test_multi_target_detection_order_does_not_trigger_track_id_rollback(self):
        """Track IDs are identifiers; detector row order is not an ID order."""

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "multi_target_order.csv"
            write_csv(path, [
                [0, 10, "tracking", 1, 49, 0, 1, 2, 2, 1, "", 0.1, 2],
                [0, 10, "tracking", 0, 49, 0, 1, 2, 2, 1, "", 0.2, 1],
            ])
            report = report_for(path)

        self.assertEqual(report["report_status"], "PASS")
        self.assertFalse(any(item["code"] == "track_id_rollback" for item in report["errors"]))

    def test_tracking_distribution_and_switches(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "states.csv"
            write_csv(path, [
                [0, 10, "detecting", 0, 50, 0, 1, 1, 0, 0, "solve_pnp_failed", "", 1],
                [1, 20, "tracking", 1, 49, 0, 1, 1, 1, 1, "", 0.1, 1],
                [2, 30, "temp_lost", 1, 50, 0, 1, 0, 0, 0, "", "", 2],
                [3, 40, "lost", 0, 50, 0, 1, 0, 0, 0, "", "", 2],
            ])
            report = report_for(path)
        self.assertEqual(report["tracker_target"]["tracking_state_distribution"], {"detecting": 1, "lost": 1, "temp_lost": 1, "tracking": 1})
        self.assertEqual(report["tracker_target"]["target_switch_count"], 1)

    def test_nonzero_fire_is_safety_failure(self):
        report = report_for(FIXTURES / "anomaly.csv")
        self.assertEqual(report["report_status"], "FAIL")
        self.assertGreater(report["safety"]["nonzero_fire_command_count"], 0)

    def test_test_only_false_is_failure(self):
        report = report_for(FIXTURES / "anomaly.csv")
        self.assertGreater(len(report["safety"]["test_only_false_frames"]), 0)

    def test_reports_include_boundary_and_units(self):
        report = report_for(FIXTURES / "normal.csv")
        markdown = render_markdown(report)
        self.assertIn("`real_hit_rate_computed`: false", markdown)
        self.assertIn("External position angle: degree", markdown)
        self.assertEqual(report["evidence_boundary"]["hardware_validation"], False)

    def test_empty_data_rows_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "header_only.csv"
            path.write_text(",".join(HEADER) + "\n", encoding="utf-8")
            report = report_for(path)
        self.assertEqual(report["coverage"]["csv_total_rows"], 0)
        self.assertEqual(report["status"], "FAIL")

    def test_metadata_allowlist_redacts_absolute_paths(self):
        analysis = analyze_csv(FIXTURES / "normal.csv", {"source_label": r"C:\private\run.csv", "secret": "ignored"})
        report = build_report(analysis)
        self.assertEqual(report["metadata"]["source_label"], "run.csv")
        self.assertNotIn("secret", report["metadata"])

    def test_cli_writes_reports_for_bad_input_and_nonzero_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            json_path = Path(directory) / "report.json"
            md_path = Path(directory) / "report.md"
            result = subprocess.run([sys.executable, str(SCRIPT), "--input-csv", str(FIXTURES / "anomaly.csv"), "--json-report", str(json_path), "--markdown-report", str(md_path)], check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertTrue(json_path.exists())
            self.assertTrue(md_path.exists())
            self.assertEqual(json.loads(json_path.read_text(encoding="utf-8"))["report_status"], "FAIL")

    def test_fifo_input_is_rejected_without_blocking(self):
        with tempfile.TemporaryDirectory() as directory:
            fifo = Path(directory) / "input.fifo"
            try:
                os.mkfifo(fifo)
            except (AttributeError, OSError) as exc:
                self.skipTest(f"FIFO unavailable: {exc}")
            report = report_for(fifo)
        self.assertEqual(report["report_status"], "FAIL")
        self.assertTrue(any(item["code"] == "input_open_error" for item in report["errors"]))

    def test_fifo_report_output_is_rejected_without_opening_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            write_csv(input_path, [[0, 10, "lost", 0, 50, 0, 1]])
            fifo = root / "report.fifo"
            try:
                os.mkfifo(fifo)
            except (AttributeError, OSError) as exc:
                self.skipTest(f"FIFO unavailable: {exc}")
            with self.assertRaisesRegex(OSError, "regular file"):
                write_reports(report_for(input_path), json_path=fifo, input_path=input_path)

    def _assert_input_alias_rejected(self, output_kind, alias_kind):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            shutil.copyfile(FIXTURES / "normal.csv", input_path)
            original = input_path.read_bytes()
            if alias_kind == "same":
                output_path = input_path
            elif alias_kind == "symlink":
                output_path = root / f"{output_kind}-symlink.out"
                try:
                    output_path.symlink_to(input_path)
                except (OSError, NotImplementedError) as exc:
                    self.skipTest(f"symlink unavailable: {exc}")
            elif alias_kind == "hardlink":
                output_path = root / f"{output_kind}-hardlink.out"
                try:
                    os.link(input_path, output_path)
                except OSError as exc:
                    self.skipTest(f"hardlink unavailable: {exc}")
            else:
                raise AssertionError(f"unknown alias kind: {alias_kind}")

            option = "--json-report" if output_kind == "json" else "--markdown-report"
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--input-csv", str(input_path), option, str(output_path)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            self.assertEqual(input_path.read_bytes(), original)

    def test_cli_rejects_json_path_equal_to_input(self):
        self._assert_input_alias_rejected("json", "same")

    def test_cli_rejects_markdown_path_equal_to_input(self):
        self._assert_input_alias_rejected("markdown", "same")

    def test_cli_rejects_json_symlink_to_input(self):
        self._assert_input_alias_rejected("json", "symlink")

    def test_cli_rejects_markdown_symlink_to_input(self):
        self._assert_input_alias_rejected("markdown", "symlink")

    def test_cli_rejects_json_hardlink_to_input(self):
        self._assert_input_alias_rejected("json", "hardlink")

    def test_cli_rejects_markdown_hardlink_to_input(self):
        self._assert_input_alias_rejected("markdown", "hardlink")

    def test_cli_rejects_json_and_markdown_aliases(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            shutil.copyfile(FIXTURES / "normal.csv", input_path)
            original = input_path.read_bytes()
            report_path = root / "same-report.out"
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--input-csv",
                    str(input_path),
                    "--json-report",
                    str(report_path),
                    "--markdown-report",
                    str(report_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            self.assertFalse(report_path.exists())
            self.assertEqual(input_path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()

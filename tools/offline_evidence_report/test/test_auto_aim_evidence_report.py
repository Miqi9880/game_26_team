import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "offline_evidence_report" / "auto_aim_evidence_report.py"
FIXTURES = ROOT / "tools" / "offline_evidence_report" / "fixtures"
sys.path.insert(0, str(ROOT))

from tools.offline_evidence_report.auto_aim_evidence_report import analyze_csv, build_report, markdown_report  # noqa: E402

def render_markdown(source):
    return markdown_report(build_report(source) if not isinstance(source, dict) else source)

def report_for(path):
    return build_report(analyze_csv(path))


HEADER = [
    "frame", "stamp_ns", "tracking_state", "selected", "target_lock", "fire_command", "test_only",
    "detection_count", "valid_pnp_count", "pnp_valid", "pnp_failure", "reprojection_error_px", "track_id",
]


def write_csv(path, rows, header=HEADER):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            values = list(row)
            if len(values) < len(header):
                values.extend([""] * (len(header) - len(values)))
            writer.writerow(values)


class EvidenceReportTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

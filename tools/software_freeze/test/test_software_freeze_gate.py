from __future__ import annotations

import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path

from tools.software_freeze.software_freeze_gate import (
    CommandResult,
    SAFE_DEFAULTS,
    _parse_ros_safety_output,
    _ros_safety_status,
    _scenario_observations,
    _scenario_consistent,
    _github_observations,
    _git_observations,
    _command_record,
    _load_observations,
    _manifest_status_hint,
    _manifest_ctest_counts,
    _manifest_hash_references,
    assess_observations,
    build_input_manifest_report,
    build_report,
    create_manifest,
    scan_evidence_claims,
    write_report_bundle,
)


def passing_observations() -> dict:
    tests = [
        {"name": name, "status": "PASS"}
        for name in (
            "auto_aim_core_test",
            "offline_pipeline_test",
            "offline_tracker_replay_test",
            "offline_predictor_test",
            "offline_ballistic_test",
            "offline_scenario_benchmark_test",
            "pnp_stage_test",
            "raw_armor_detector_test",
            "ros_image_adapter_test",
            "ros_backend_test",
            "vision_time_alignment_test",
            "ros_safety_integration_test",
            "camera_calibration_test",
            "auto_aim_offline_csv_test",
            "auto_aim_offline_cli_test",
            "auto_aim_scenario_benchmark_cli_test",
            "serial_protocol_loopback_test",
            "robot_ctrl_safety_test",
            "preflight_analyzer_test",
            "calibration_dataset_test",
            "calibration_dataset_ros_test",
            "fake_ros_publishers_test",
            "process_contract_test",
            "report_test",
        )
    ]
    run = {
        "status": "PASS",
        "synthetic": True,
        "test_only": True,
        "production_ready": False,
        "software_only_synthetic_benchmark": True,
        "origin_assumption": "synthetic_muzzle_frame",
        "csv_invariants": {"status": "PASS", "rows": 1},
        "safety": SAFE_DEFAULTS.copy(),
        "files": {
            name: {
                "status": "PASS",
                "sha256": hashlib.sha256(f"{name}\n".encode()).hexdigest(),
                "size_bytes": len(f"{name}\n"),
                "bytes_equal": True,
            }
            for name in ("benchmark.csv", "summary.json", "summary.md")
        },
    }
    return {
        "git": {
            "head_sha": "a" * 40,
            "origin_main_sha": "a" * 40,
            "branch": "feat/software-freeze-gate",
            "worktree_clean": True,
            "head_matches_origin_main": True,
            "candidate_base_matches_origin_main": True,
            "diff_check": {"status": "PASS", "exit_code": 0},
        },
        "github": {
            "status": "PASS",
            "open_prs": [],
            "prs": {
                str(number): {
                    "number": number,
                    "state": "MERGED",
                    "isDraft": False,
                    "reviewDecision": "APPROVED" if number == 29 else "REVIEW_REQUIRED",
                }
                for number in (27, 28, 29, 30)
            },
        },
        "build": {"status": "PASS", "exit_code": 0},
        "cpp_tests": {"status": "PASS", "tests": tests, "total": 200, "failed": 0, "errors": 0, "skipped": 0},
        "python_tests": {name: {"status": "PASS"} for name in ("qualification", "offline_evidence_report", "offline_evidence_bundle", "orin_unavailable")},
        "scenario": {"runs": [run, json.loads(json.dumps(run))]},
        "cli_help": {
            "auto_aim_scenario_benchmark_help": {"status": "PASS"},
            "auto_aim_offline_help": {"status": "PASS"},
        },
        "runtime": {
            "openvino_python": {"status": "UNAVAILABLE", "reason": "test fixture has no runtime"},
        },
        "camera_preflight": {"status": "NOT_VERIFIED", "reason": "fixture has no camera"},
        "evidence": {"status": "PASS", "production_claim_rejected": True, "claims": {"production_ready": False}},
        "ros_safety": {
            "rounds": [
                {"status": "PASS", "motion_nonzero": False, **SAFE_DEFAULTS.copy()}
                for _ in range(3)
            ]
        },
        "safety_defaults": {"values": SAFE_DEFAULTS.copy()},
        "hardware": {name: {"status": "NOT_VERIFIED"} for name in ("model", "calibration", "camera", "orin", "cdc", "gimbal", "firing")},
    }


class SoftwareFreezeGateTests(unittest.TestCase):
    def test_clean_all_pass_is_ready_candidate(self) -> None:
        report = build_report(passing_observations())
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")
        self.assertEqual(report["blockers"], [])
        self.assertEqual(report["observations"]["github"]["open_prs"], [])
        self.assertEqual(report["safety_boundary"], SAFE_DEFAULTS)

    def test_dirty_worktree_blocks(self) -> None:
        observations = passing_observations()
        observations["git"]["worktree_clean"] = False
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("WORKTREE_DIRTY", report["blockers"])

    def test_diff_check_failure_blocks(self) -> None:
        observations = passing_observations()
        observations["git"]["diff_check"] = {"status": "FAIL", "exit_code": 2}
        report = build_report(observations)
        self.assertIn("GIT_DIFF_CHECK_FAILED", report["blockers"])

    def test_head_mismatch_blocks(self) -> None:
        observations = passing_observations()
        observations["git"]["head_matches_origin_main"] = False
        observations["git"]["candidate_base_matches_origin_main"] = False
        report = build_report(observations)
        self.assertIn("HEAD_NOT_AT_CHECKED_MAIN", report["blockers"])

    def test_remote_main_sha_is_used_for_ancestry_not_stale_local_ref(self) -> None:
        remote_sha = "b" * 40
        calls: list[tuple[str, ...]] = []

        def fake_runner(argv, **_kwargs):
            command = tuple(str(item) for item in argv)
            calls.append(command)
            if command[:3] == ("git", "rev-parse", "HEAD"):
                return CommandResult(0, "a" * 40 + "\n")
            if command[:3] == ("git", "branch", "--show-current"):
                return CommandResult(0, "feature\n")
            if command[:3] == ("git", "status", "--porcelain"):
                return CommandResult(0, "")
            if command == ("git", "diff", "--check"):
                return CommandResult(0)
            if command == ("git", "ls-remote", "origin", "refs/heads/main"):
                return CommandResult(0, f"{remote_sha}\trefs/heads/main\n")
            if command == ("git", "merge-base", "--is-ancestor", remote_sha, "HEAD"):
                return CommandResult(1, "", "not an ancestor")
            return CommandResult(127, "", "unexpected command")

        result = _git_observations(Path("/tmp/fake-worktree"), fake_runner, [])
        self.assertFalse(result["candidate_base_matches_origin_main"])
        self.assertIn(("git", "merge-base", "--is-ancestor", remote_sha, "HEAD"), calls)

    def test_command_output_redacts_host_and_normalizes_timings(self) -> None:
        first = _command_record(
            "probe",
            ["echo", "ok"],
            CommandResult(0, "Site: LAPTOP-A\n1/1 Test #1: probe .... Passed    1.23 sec\nRan 2 tests in 0.21s\n"),
            cwd=Path("/tmp/fake-worktree"),
        )
        second = _command_record(
            "probe",
            ["echo", "ok"],
            CommandResult(0, "Site: LAPTOP-B\n1/1 Test #1: probe .... Passed    9.87 sec\nRan 2 tests in 8.76s\n"),
            cwd=Path("/tmp/fake-worktree"),
        )
        self.assertEqual(first["stdout"], second["stdout"])
        self.assertIn("<duration>", first["stdout"])
        self.assertIn("<redacted>", first["stdout"])

    def test_build_command_does_not_leak_scratch_path(self) -> None:
        from tools.software_freeze.software_freeze_gate import _display_command

        command = _display_command(["bash", "-lc", "colcon build --build-base /tmp/private-build"])
        self.assertNotIn("/tmp/private-build", command)
        self.assertIn("<path>", command)

    def test_open_draft_pr_is_informational(self) -> None:
        observations = passing_observations()
        observations["github"]["open_prs"] = [{"number": 41, "isDraft": True, "state": "OPEN"}]
        report = build_report(observations)
        self.assertNotIn("OPEN_FUNCTION_PR_41", report["blockers"])

    def test_historical_pr_state_is_not_a_blocker(self) -> None:
        observations = passing_observations()
        observations["github"]["prs"]["29"]["state"] = "OPEN"
        report = build_report(observations)
        self.assertIsNone(report["blocker"])
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")

    def test_github_unavailable_is_recorded_without_guessing_no_pr(self) -> None:
        observations = passing_observations()
        observations["github"] = {"status": "UNAVAILABLE", "open_prs": []}
        report = build_report(observations)
        self.assertNotIn("GITHUB_STATUS_UNAVAILABLE", report["blockers"])
        self.assertNotEqual(report["observations"]["github"]["status"], "PASS")

    def test_missing_github_open_prs_is_not_inferred(self) -> None:
        observations = passing_observations()
        observations["github"].pop("open_prs")
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")
        self.assertNotIn("GITHUB_OPEN_PR_PARSE_FAILED", report["blockers"])

    def test_single_test_failure_blocks(self) -> None:
        observations = passing_observations()
        next(item for item in observations["cpp_tests"]["tests"] if item["name"] == "offline_pipeline_test")["status"] = "FAIL"
        report = build_report(observations)
        self.assertIn("TEST_offline_pipeline_test_NOT_PASS", report["blockers"])

    def test_cli_help_failure_blocks(self) -> None:
        observations = passing_observations()
        observations["cli_help"]["auto_aim_offline_help"] = {"status": "FAIL", "exit_code": 127}
        report = build_report(observations)
        self.assertIn("CLI_HELP_auto_aim_offline_help_FAILED", report["blockers"])

    def test_flaky_ros_rounds_block(self) -> None:
        observations = passing_observations()
        observations["ros_safety"]["rounds"] = [
            {"status": "FAIL", "motion_nonzero": False, **SAFE_DEFAULTS.copy()},
            {"status": "PASS", "motion_nonzero": False, **SAFE_DEFAULTS.copy()},
            {"status": "PASS", "motion_nonzero": False, **SAFE_DEFAULTS.copy()},
        ]
        report = build_report(observations)
        self.assertIn("ROS_SAFETY_ROUNDS_NOT_CONSISTENT", report["blockers"])
        self.assertIn("inconsistent/flaky", report["observations"]["ros_safety"]["failures"][1])

    def test_scenario_missing_metadata_blocks_fail_closed(self) -> None:
        observations = passing_observations()
        for run in observations["scenario"]["runs"]:
            run.pop("status", None)
            run.pop("csv_invariants", None)
            run.pop("origin_assumption", None)
            run.pop("safety", None)
            for record in run["files"].values():
                record.pop("status", None)
                record.pop("sha256", None)
                record.pop("size_bytes", None)
                record.pop("bytes_equal", None)
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("SCENARIO_BENCHMARK_NOT_DETERMINISTIC", report["blockers"])

    def test_hardware_statuses_never_claim_readiness(self) -> None:
        for status in ("PASS", "FAIL", "BLOCKED"):
            observations = passing_observations()
            observations["hardware"]["model"] = {"status": status}
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", status)
            self.assertTrue(any(item.startswith("HARDWARE_MODEL_") for item in report["blockers"]))

    def test_python_unavailable_blocks_except_orin_gap(self) -> None:
        for group in ("qualification", "offline_evidence_report", "offline_evidence_bundle"):
            observations = passing_observations()
            observations["python_tests"][group] = {"status": "UNAVAILABLE"}
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", group)
        observations = passing_observations()
        observations["python_tests"]["orin_unavailable"] = {"status": "UNAVAILABLE"}
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")

    def test_ros_safety_motion_alias_and_missing_fields_block(self) -> None:
        observations = passing_observations()
        observations["ros_safety"]["rounds"] = [
            {"status": "PASS", **SAFE_DEFAULTS.copy(), "yaw_vel_rad_s": 1}
            for _ in range(3)
        ]
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")

    def test_ros_safety_motion_nonzero_is_fail_closed_for_truthy_aliases(self) -> None:
        for value in (True, 1, "true", "yes", "invalid"):
            observations = passing_observations()
            observations["ros_safety"]["rounds"] = [
                {"status": "PASS", **SAFE_DEFAULTS.copy(), "motion_nonzero": value}
                for _ in range(3)
            ]
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", value)

    def test_invalid_git_sha_blocks_even_with_ancestor_flag(self) -> None:
        observations = passing_observations()
        observations["git"].update({
            "head_sha": "not-a-sha",
            "origin_main_sha": "still-not-a-sha",
            "candidate_base_matches_origin_main": True,
        })
        report = build_report(observations)
        self.assertIn("GIT_HEAD_INVALID", report["blockers"])
        self.assertIn("ORIGIN_MAIN_INVALID", report["blockers"])

    def test_bundle_failure_requires_claim_rejection_signal(self) -> None:
        observations = passing_observations()
        observations["evidence"] = {
            "status": "PASS",
            "production_claim_rejected": True,
            "bundle_status": "FAIL",
            "bundle_report_status": "FAIL",
            "bundle_diagnostics": {"errors": [{"code": "other_failure"}]},
        }
        report = build_report(observations)
        self.assertIn("EVIDENCE_BOUNDARY_FAILED", report["blockers"])

    def test_nested_evidence_claims_are_scanned(self) -> None:
        for nested in (
            {"production_ready": True},
            {"bundle": {"hardware_validation": True}},
            {"files": [{"allow_fire": True}]},
            {"safety": {"fire_command": 2}},
        ):
            observations = passing_observations()
            observations["evidence"].update(nested)
            report = build_report(observations)
            self.assertIn("EVIDENCE_BOUNDARY_FAILED", report["blockers"], nested)

    def test_evidence_claim_aliases_cannot_hide_in_camel_case(self) -> None:
        for claim in (
            {"productionReady": True},
            {"hardwareValidation": True},
            {"serialEnabled": True},
            {"testOnly": False},
            {"dryRun": False},
            {"pitchAcc": 1},
            {"motionNonzero": True},
            {"diagnostic": "productionReady=true"},
            {"diagnostic": "serialEnabled=true"},
            {"diagnostic": "dryRun=false"},
        ):
            self.assertEqual(scan_evidence_claims(claim)["status"], "FAIL", claim)
            observations = passing_observations()
            observations["evidence"] = {"status": "PASS", "production_claim_rejected": True, "claims": claim}
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", claim)

    def test_ros_safety_transcript_explicit_unsafe_values_are_parsed(self) -> None:
        parsed = _parse_ros_safety_output(
            "ros_safety_integration_test: fire=2 yaw_vel_rad_s=0.5 "
            "pitch_acc_rad_s2:=1 dry_run:=false\n"
        )
        self.assertEqual(parsed["fire_command"], 2)
        self.assertEqual(parsed["yaw_vel"], 0.5)
        self.assertEqual(parsed["pitch_acc"], 1)
        self.assertFalse(parsed["dry_run"])
        status, failures = _ros_safety_status({"rounds": [{"status": "PASS", **parsed}] * 3})
        self.assertEqual(status, "FAIL")
        self.assertTrue(failures)
        malformed = _parse_ros_safety_output("fire_command=bogus")
        self.assertTrue(malformed["_parse_errors"])
        missing = _parse_ros_safety_output("")
        status, failures = _ros_safety_status({"rounds": [{"status": "PASS", **missing}] * 3})
        self.assertEqual(status, "FAIL")
        self.assertTrue(any("omitted safety fields" in item for item in failures))
        repeated = _parse_ros_safety_output("fire=2 fire=0 dry_run=false dry_run=true")
        self.assertTrue(repeated["_parse_errors"] or repeated["fire_command"] != 0 or repeated["dry_run"] is not True)
        status, _ = _ros_safety_status({"rounds": [{"status": "PASS", **repeated}] * 3})
        self.assertEqual(status, "FAIL")
        explicit_safe = _parse_ros_safety_output(
            "serial_enabled=false dry_run=true allow_fire=false fire_command=0 "
            "yaw_vel=0 pitch_vel=0 yaw_acc=0 pitch_acc=0 motion_nonzero=false"
        )
        status, failures = _ros_safety_status({"rounds": [{"status": "PASS", **explicit_safe}] * 3})
        self.assertEqual(status, "PASS")
        self.assertEqual(failures, [])
        observations = passing_observations()
        observations["ros_safety"]["rounds"] = [{"status": "PASS"} for _ in range(3)]
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")

    def test_evidence_boundary_aliases_and_control_claims_block(self) -> None:
        for claim in (
            {"dry_run": False},
            {"yaw_vel_rad_s": 1},
            {"pitch_acc_rad_s2": 1},
            {"absolute_command_valid": True},
            {"relative_angle_as_robotctrl": True},
            {"gimbal_origin_as_muzzle": True},
            {"serial_enabled": True},
        ):
            observations = passing_observations()
            observations["evidence"] = {
                "status": "PASS",
                "production_claim_rejected": True,
                "claims": claim,
            }
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", claim)
            self.assertEqual(scan_evidence_claims(claim)["status"], "FAIL")

    def test_malformed_open_pr_is_informational(self) -> None:
        observations = passing_observations()
        observations["github"]["open_prs"] = [{}]
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")
        self.assertNotIn("OPEN_PR_UNIDENTIFIABLE", report["blockers"])

    def test_github_malformed_open_pr_record_is_unavailable(self) -> None:
        def fake_runner(argv, **_kwargs):
            if list(argv)[:3] == ["gh", "pr", "list"]:
                return CommandResult(0, '[{"number": 34}, "malformed"]')
            return CommandResult(127, "", "unexpected command")

        result = _github_observations("example/repo", fake_runner, [])
        self.assertEqual(result["status"], "UNAVAILABLE")

    def test_explicit_scenario_root_must_be_new_and_safe(self) -> None:
        def fake_runner(argv, **_kwargs):
            command = " ".join(str(item) for item in argv)
            if "--help" in command:
                return CommandResult(0, "usage")
            if "--scenario all" in command:
                output = Path(command.split("--output-dir", 1)[1].strip().strip("'\""))
                output.mkdir(parents=True, exist_ok=True)
                (output / "benchmark.csv").write_text(
                    "synthetic,test_only,production_ready,origin_assumption,fire_command,"
                    "yaw_vel_rad_s,pitch_vel_rad_s,yaw_acc_rad_s2,pitch_acc_rad_s2,"
                    "serial_enabled,dry_run,allow_fire,ballistic_reason\n"
                    "true,true,false,synthetic_muzzle_frame,0,0,0,0,0,false,true,false,none\n",
                    encoding="utf-8",
                )
                (output / "summary.json").write_text(json.dumps({
                    "synthetic": True,
                    "test_only": True,
                    "production_ready": False,
                    "software_only_synthetic_benchmark": True,
                    "safety": SAFE_DEFAULTS.copy(),
                }), encoding="utf-8")
                (output / "summary.md").write_text("synthetic benchmark\n", encoding="utf-8")
                return CommandResult(0, "synthetic=true")
            return CommandResult(127, "", "unexpected command")

        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent) / "existing"
            root.mkdir()
            (root / "sentinel").write_text("keep", encoding="utf-8")
            result = _scenario_observations(Path(parent), fake_runner, [], scenario_root=root)
            self.assertEqual(result["status"], "PASS")
            self.assertEqual((root / "sentinel").read_text(encoding="utf-8"), "keep")
            occupied = root / "occupied"
            occupied.mkdir()
            result = _scenario_observations(Path(parent), fake_runner, [], scenario_root=root)
            self.assertEqual(result["status"], "FAIL")
            self.assertIn("unsafe scenario output directory a", result["failures"][0])
            target = Path(parent) / "target"
            target.mkdir()
            link = Path(parent) / "link"
            os.symlink(target, link, target_is_directory=True)
            result = _scenario_observations(Path(parent), fake_runner, [], scenario_root=link)
            self.assertEqual(result["status"], "FAIL")

    def test_colcon_output_normalizes_ms_and_order(self) -> None:
        first = _command_record(
            "colcon_test", ["colcon", "test"],
            CommandResult(0, "B (308 ms)\nA (1.2 s)\n"),
        )
        second = _command_record(
            "colcon_test", ["colcon", "test"],
            CommandResult(0, "A (9.8 s)\nB (999 ms)\n"),
        )
        self.assertEqual(first["stdout"], second["stdout"])

    def test_scenario_byte_mismatch_blocks(self) -> None:
        observations = passing_observations()
        observations["scenario"]["runs"][1]["files"]["summary.md"]["sha256"] = "0" * 64
        report = build_report(observations)
        self.assertIn("SCENARIO_BENCHMARK_NOT_DETERMINISTIC", report["blockers"])

    def test_scenario_safety_and_origin_claims_block(self) -> None:
        observations = passing_observations()
        observations["scenario"]["runs"][0]["software_only_synthetic_benchmark"] = False
        observations["scenario"]["runs"][0]["safety"]["yaw_vel"] = 1
        report = build_report(observations)
        self.assertIn("SCENARIO_BENCHMARK_NOT_DETERMINISTIC", report["blockers"])

    def test_nested_scenario_production_claim_blocks(self) -> None:
        observations = passing_observations()
        observations["scenario"]["runs"][0]["evidence_boundary"] = {
            "nested": {"production_ready": "true"}
        }
        report = build_report(observations)
        self.assertIn("SCENARIO_BENCHMARK_NOT_DETERMINISTIC", report["blockers"])

    def test_zero_test_summary_is_not_run(self) -> None:
        from tools.software_freeze.software_freeze_gate import _test_result_from_text

        result = _test_result_from_text("python_check", "Ran 0 tests in 0.00s\nOK\n", 0)
        self.assertEqual(result["status"], "NOT_RUN")

    def test_required_test_result_counts_fail_closed(self) -> None:
        observations = passing_observations()
        observations["cpp_tests"]["total"] = 0
        observations["cpp_tests"]["failed"] = 0
        observations["cpp_tests"]["errors"] = 0
        observations["cpp_tests"]["skipped"] = 0
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("CPP_TEST_COUNT_INVALID", report["blockers"])
        observations["cpp_tests"]["tests"] = []
        report = build_report(observations)
        self.assertIn("auto_aim_core_test", report["not_run"])
        observations = passing_observations()
        observations["cpp_tests"].pop("total")
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("CPP_TEST_COUNT_INVALID", report["blockers"])

    def test_pr34_report_test_is_required(self) -> None:
        observations = passing_observations()
        observations["cpp_tests"]["tests"] = [
            item for item in observations["cpp_tests"]["tests"] if item["name"] != "report_test"
        ]
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("report_test", report["not_run"])

    def test_pr34_package_is_selected_by_live_build_and_test(self) -> None:
        # Keep the command contract explicit: the ROS message-E2E package must
        # not be silently excluded from the normal freeze build/test probes.
        source = Path(__file__).resolve().parents[1].joinpath("software_freeze_gate.py").read_text(encoding="utf-8")
        self.assertGreaterEqual(source.count("auto_aim_ros_e2e"), 2)

    def test_injected_runner_uses_ros2_run_and_checks_scenario_files(self) -> None:
        calls: list[list[str]] = []
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            scenario_root = root / "scenario"
            scenario_root.mkdir()

            def fake_runner(argv, cwd=None, timeout=600.0):
                command = " ".join(str(item) for item in argv)
                calls.append([str(item) for item in argv])
                if "--help" in command:
                    return CommandResult(0, "usage: auto_aim_scenario_benchmark")
                if "--scenario all" in command:
                    suffix = "a" if str(scenario_root / "a") in command else "b"
                    output = scenario_root / suffix
                    output.mkdir(parents=True, exist_ok=True)
                    (output / "benchmark.csv").write_text(
                        "synthetic,test_only,production_ready,origin_assumption,fire_command,"
                        "yaw_vel_rad_s,pitch_vel_rad_s,yaw_acc_rad_s2,pitch_acc_rad_s2,"
                        "serial_enabled,dry_run,allow_fire,ballistic_reason\n"
                        "true,true,false,synthetic_muzzle_frame,0,0,0,0,0,false,true,false,none\n",
                        encoding="utf-8",
                    )
                    (output / "summary.json").write_text(json.dumps({
                        "synthetic": True,
                        "test_only": True,
                        "production_ready": False,
                        "software_only_synthetic_benchmark": True,
                        "safety": SAFE_DEFAULTS.copy(),
                    }, sort_keys=True), encoding="utf-8")
                    (output / "summary.md").write_text("synthetic benchmark\n", encoding="utf-8")
                    return CommandResult(0, "synthetic=true")
                return CommandResult(127, "", "unexpected fake command")

            result = _scenario_observations(root, fake_runner, [], scenario_root=scenario_root)
            self.assertEqual(result["status"], "PASS")
            self.assertEqual(len(result["runs"]), 2)
            self.assertTrue(all("ros2 run auto_aim_ros2 auto_aim_scenario_benchmark" in " ".join(call) for call in calls))

    def test_production_claim_and_nonzero_fire_are_rejected(self) -> None:
        self.assertEqual(scan_evidence_claims({"production_ready": True})["status"], "FAIL")
        observations = passing_observations()
        observations["evidence"] = {"status": "PASS", "production_claim_rejected": True, "claims": {"fire_command": 2}}
        report = build_report(observations)
        self.assertIn("EVIDENCE_BOUNDARY_FAILED", report["blockers"])

    def test_malformed_claim_rejection_diagnostics_cannot_exempt_claim(self) -> None:
        observations = passing_observations()
        observations["evidence"] = {
            "status": "PASS",
            "production_claim_rejected": True,
            "bundle_diagnostics": "not-an-object",
            "claims": {"production_ready": True},
        }
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "BLOCKED")
        self.assertIn("EVIDENCE_BOUNDARY_FAILED", report["blockers"])

    def test_missing_model_and_runtime_are_not_verified_not_production_pass(self) -> None:
        observations = passing_observations()
        observations["hardware"]["model"] = {"status": "UNAVAILABLE", "reason": "missing XML/BIN"}
        observations["runtime"] = {"openvino_python": {"status": "UNAVAILABLE", "reason": "ImportError"}}
        report = build_report(observations)
        self.assertEqual(report["software_candidate_status"], "READY_CANDIDATE")
        self.assertIn("runtime.openvino_python", report["not_verified"])
        self.assertEqual(report["observations"]["hardware"]["model"]["status"], "UNAVAILABLE")

    def test_nonzero_safety_default_blocks(self) -> None:
        observations = passing_observations()
        observations["safety_defaults"]["values"]["fire_command"] = 1
        report = build_report(observations)
        self.assertIn("SAFETY_DEFAULTS_INVALID", report["blockers"])

    def test_missing_required_top_level_observations_block(self) -> None:
        for key, blocker in (("hardware", "HARDWARE_STATUS_UNAVAILABLE"), ("safety_defaults", "SAFETY_DEFAULTS_UNAVAILABLE")):
            observations = passing_observations()
            observations.pop(key)
            report = build_report(observations)
            self.assertEqual(report["software_candidate_status"], "BLOCKED", key)
            self.assertIn(blocker, report["blockers"])

    def test_missing_checks_are_not_run_and_block(self) -> None:
        observations = passing_observations()
        observations["build"] = {"status": "NOT_RUN"}
        report = build_report(observations)
        self.assertIn("offline_build", report["not_run"])
        self.assertEqual(report["software_candidate_status"], "BLOCKED")

    def test_hardware_unverified_is_non_blocking(self) -> None:
        report = build_report(passing_observations())
        self.assertIn("model", report["non_blocking_hardware"])
        self.assertEqual(report["observations"]["hardware"]["orin"]["status"], "NOT_VERIFIED")

    def test_manifest_and_output_are_reproducible(self) -> None:
        report = build_report(passing_observations())
        files = {"a.json": b"{}\n", "b.md": b"stable\n"}
        first = create_manifest(report, files)
        second = create_manifest(report, files)
        self.assertEqual(first, second)
        with tempfile.TemporaryDirectory() as parent:
            output = Path(parent) / "bundle"
            manifest = write_report_bundle(report, output)
            self.assertEqual(manifest["artifacts"][0]["sha256"], hashlib.sha256((output / manifest["artifacts"][0]["path"]).read_bytes()).hexdigest())
            with self.assertRaises(ValueError):
                write_report_bundle(report, output)

    def test_symlink_output_is_rejected_without_mutation(self) -> None:
        if not hasattr(os, "symlink"):
            self.skipTest("symlink unavailable")
        report = build_report(passing_observations())
        with tempfile.TemporaryDirectory() as parent:
            target = Path(parent) / "target"
            target.mkdir()
            link = Path(parent) / "link"
            os.symlink(target, link, target_is_directory=True)
            with self.assertRaises(ValueError):
                write_report_bundle(report, link)
            self.assertEqual(list(target.iterdir()), [])

    def test_explicit_input_manifest_records_hashes_and_liveness(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report_path = root / "e2e.json"
            ctest_path = root / "Test.xml"
            model_path = root / "model.xml"
            model_path.write_text("model fixture\n", encoding="utf-8")
            model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
            report_path.write_text(json.dumps({
                "schema_version": 1,
                "status": "PASS",
                "baseline_main": "b" * 40,
                "candidate_commit": "a" * 40,
                "synthetic": True,
                "test_only": True,
                "production_ready": False,
                "safety_assertions": SAFE_DEFAULTS.copy(),
                "node_liveness": {
                    "alive_before_sampling": True,
                    "alive_during_sampling": True,
                    "alive_after_sampling": True,
                    "expected_exit_code": 0,
                    "observed_exit_code": 0,
                    "exit_code_matches": True,
                },
                "counts": {"PASS": 1, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0},
                "cases": [{
                    "name": "e2e", "status": "PASS",
                    "node_liveness_applicable": True,
                    "node_liveness": {
                        "alive_before_sampling": True,
                        "alive_during_sampling": True,
                        "alive_after_sampling": True,
                        "expected_exit_code": 0,
                        "observed_exit_code": 0,
                        "exit_code_matches": True,
                    },
                }],
                "model_xml_sha256": model_hash,
            }, sort_keys=True), encoding="utf-8")
            ctest_path.write_text(
                "<Site><Testing><Test Status=\"passed\"><Name>e2e</Name></Test></Testing></Site>",
                encoding="utf-8",
            )
            manifest = {
                "schema": "software-freeze-inputs",
                "schema_version": 1,
                "candidate": {
                    "head": "a" * 40,
                    "main_baseline": "b" * 40,
                    "branch": "codex/software-freeze",
                    "worktree_clean": True,
                    "git_object_verified": True,
                    "origin_main_verified": True,
                    "ancestry_verified": True,
                },
                "required_kinds": ["ros_e2e"],
                "inputs": [{"id": "e2e", "kind": "ros_e2e", "path": str(report_path),
                            "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
                            "ctest_xml": str(ctest_path),
                            "ctest_xml_sha256": hashlib.sha256(ctest_path.read_bytes()).hexdigest()}],
                "artifacts": [{"role": "model_xml", "path": str(model_path), "sha256": model_hash}],
            }
            assessed = build_input_manifest_report(manifest, manifest_path=root / "inputs.json")
            self.assertEqual(assessed["software_candidate_status"], "READY_CANDIDATE")
            self.assertEqual(assessed["inputs"][0]["sha256"], hashlib.sha256(report_path.read_bytes()).hexdigest())
            self.assertEqual(assessed["artifact_hashes"]["model_xml"], model_hash)
            self.assertTrue(assessed["consistency"]["artifact_hash_consistent"])

    def test_explicit_input_manifest_liveness_and_hash_mismatch_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report_path = root / "ros-e2e.json"
            report_path.write_text(json.dumps({
                "schema_version": 1,
                "status": "PASS",
                "baseline_main": "b" * 40,
                "candidate_commit": "a" * 40,
                "safety_assertions": SAFE_DEFAULTS.copy(),
            }), encoding="utf-8")
            base = {
                "schema": "software-freeze-inputs", "schema_version": 1,
                "candidate": {"head": "a" * 40, "main_baseline": "b" * 40,
                               "branch": "feature", "worktree_clean": True,
                               "git_object_verified": True, "origin_main_verified": True,
                               "ancestry_verified": True},
                "required_kinds": ["ros_e2e"],
                "inputs": [{"id": "e2e", "kind": "ros_e2e", "path": str(report_path),
                            "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest()}],
            }
            assessed = build_input_manifest_report(base, manifest_path=root / "inputs.json")
            self.assertEqual(assessed["software_candidate_status"], "NOT_VERIFIED")
            bad_hash = dict(base)
            bad_hash["inputs"] = [{"id": "e2e", "kind": "ros_e2e", "path": str(report_path), "sha256": "0" * 64}]
            assessed = build_input_manifest_report(bad_hash, manifest_path=root / "inputs.json")
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertTrue(any("ARTIFACT" not in item for item in assessed["blockers"]))

    def test_explicit_input_manifest_binds_declared_branch_to_repository(self) -> None:
        """A repository-backed audit must not trust an arbitrary branch label."""

        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report = self._safe_report()
            manifest = self._minimal_input_manifest(root, report)
            # The helper uses a deliberately generic branch for API-only
            # manifests.  Once a repository is supplied, the declaration must
            # agree with the branch observed by the read-only Git probes.
            manifest["candidate"]["branch"] = "declared-branch"

            def fake_runner(argv, **_kwargs):
                command = tuple(str(item) for item in argv)
                if command[:3] == ("git", "rev-parse", "HEAD"):
                    return CommandResult(0, "a" * 40 + "\n")
                if command[:3] == ("git", "branch", "--show-current"):
                    return CommandResult(0, "observed-branch\n")
                if command[:3] == ("git", "status", "--porcelain"):
                    return CommandResult(0, "")
                if command == ("git", "diff", "--check"):
                    return CommandResult(0, "")
                if command == ("git", "ls-remote", "origin", "refs/heads/main"):
                    return CommandResult(0, "" + "b" * 40 + "\trefs/heads/main\n")
                if command == ("git", "merge-base", "--is-ancestor", "b" * 40, "HEAD"):
                    return CommandResult(0, "")
                return CommandResult(127, "", "unexpected command")

            assessed = build_input_manifest_report(
                manifest, manifest_path=root / "inputs.json", repo_root=root, runner=fake_runner,
            )
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("candidate branch does not match actual Git branch", assessed["blockers"])
            self.assertEqual(assessed["git_binding"]["observed_branch"], "observed-branch")

    def test_explicit_input_manifest_rejects_contradictory_optional_liveness(self) -> None:
        """Optional E2E phase/equality fields remain fail-closed when present."""

        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report_path = root / "ros-e2e.json"
            report_path.write_text(json.dumps({
                "schema_version": 1,
                "status": "PASS",
                "synthetic": True,
                "test_only": True,
                "production_ready": False,
                "safety_assertions": SAFE_DEFAULTS.copy(),
                "node_liveness": {
                    "alive_before_sampling": True,
                    "alive_during_sampling": True,
                    "alive_after_sampling": False,
                    "expected_exit_code": 0,
                    "observed_exit_code": 0,
                    "exit_code_matches": False,
                },
            }), encoding="utf-8")
            manifest = {
                "schema": "software-freeze-inputs",
                "schema_version": 1,
                "candidate": {
                    "head": "a" * 40,
                    "main_baseline": "b" * 40,
                    "branch": "feature",
                    "worktree_clean": True,
                    "git_object_verified": True,
                    "origin_main_verified": True,
                    "ancestry_verified": True,
                },
                "required_kinds": ["ros_e2e"],
                "inputs": [{
                    "id": "e2e",
                    "kind": "ros_e2e",
                    "path": str(report_path),
                    "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
                }],
            }
            assessed = build_input_manifest_report(manifest, manifest_path=root / "inputs.json")
            self.assertIn(assessed["software_candidate_status"], {"BLOCKED", "NOT_VERIFIED"})
            self.assertEqual(assessed["inputs"][0]["status"], "NOT_VERIFIED")
            self.assertIn("node_liveness evidence missing or contradictory", assessed["inputs"][0]["failure_reasons"])

    def test_explicit_input_manifest_rejects_empty_inapplicable_sentinel(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            ctest = root / "Test.xml"
            ctest.write_text(
                "<Site><Testing><Test Status=\"passed\"><Name>non_node_pass</Name></Test></Testing></Site>",
                encoding="utf-8",
            )
            report = self._safe_report(
                node_liveness={
                    "alive_before_sampling": True,
                    "alive_during_sampling": True,
                    "alive_after_sampling": True,
                    "expected_exit_code": 0,
                    "observed_exit_code": 0,
                    "exit_code_matches": True,
                },
                counts={"PASS": 1, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0},
                cases=[{
                    "name": "non_node_pass",
                    "status": "PASS",
                    "node_liveness_applicable": False,
                    "node_liveness": {},
                }],
            )
            manifest = self._minimal_input_manifest(
                root, report, kind="ros_e2e", ctest_xml=str(ctest),
            )
            manifest["inputs"][0]["ctest_xml_sha256"] = hashlib.sha256(ctest.read_bytes()).hexdigest()
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "NOT_VERIFIED")
            self.assertIn(
                "node_liveness evidence missing or contradictory for case non_node_pass",
                assessed["inputs"][0]["failure_reasons"],
            )

    def _minimal_input_manifest(self, root: Path, report: dict, *, kind: str = "build", **declaration):
        report_path = root / f"{kind}.json"
        report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
        source = {
            "id": kind,
            "kind": kind,
            "path": str(report_path),
            "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
        }
        source.update(declaration)
        return {
            "schema": "software-freeze-inputs",
            "schema_version": 1,
            "candidate": {
                "head": "a" * 40,
                "main_baseline": "b" * 40,
                "branch": "feature",
                "worktree_clean": True,
                "git_object_verified": True,
                "origin_main_verified": True,
                "ancestry_verified": True,
            },
            "required_kinds": [kind],
            "inputs": [source],
        }

    def _safe_report(self, **extra):
        report = {
            "schema_version": 1,
            "status": "PASS",
            "baseline_main": "b" * 40,
            "candidate_commit": "a" * 40,
            "safety_assertions": SAFE_DEFAULTS.copy(),
        }
        report.update(extra)
        return report

    def test_manifest_rejects_unknown_and_non_string_kinds(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            for value in ("bogus", 123, ["build"]):
                report = self._safe_report()
                manifest = self._minimal_input_manifest(root, report, kind="build")
                manifest["inputs"][0]["kind"] = value
                assessed = build_input_manifest_report(manifest)
                self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
                self.assertTrue(any("INPUT_build_FAILED" == item for item in assessed["blockers"]))

    def test_manifest_status_alias_conflict_blocks(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            manifest = self._minimal_input_manifest(root, self._safe_report(overall_status="FAIL"))
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("conflicting status aliases", assessed["inputs"][0]["failure_reasons"])
            self.assertEqual(_manifest_status_hint({"status": "PASS", "result": "FAIL"})[0], None)

    def test_manifest_requires_nonempty_allowed_required_kinds(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            for value in (None, [], [""], ["bogus"], [123]):
                manifest = self._minimal_input_manifest(root, self._safe_report())
                if value is None:
                    manifest.pop("required_kinds")
                else:
                    manifest["required_kinds"] = value
                with self.assertRaises(ValueError):
                    build_input_manifest_report(manifest)

    def test_manifest_rejects_parent_symlink(self) -> None:
        if not hasattr(os, "symlink"):
            self.skipTest("symlink unavailable")
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            real = root / "real"
            real.mkdir()
            report_path = real / "report.json"
            report_path.write_text(json.dumps(self._safe_report()), encoding="utf-8")
            link = root / "linked"
            os.symlink(real, link, target_is_directory=True)
            manifest = self._minimal_input_manifest(root, self._safe_report())
            manifest["inputs"][0]["path"] = str(link / "report.json")
            manifest["inputs"][0]["sha256"] = hashlib.sha256(report_path.read_bytes()).hexdigest()
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("declared input path traverses a symlink", assessed["inputs"][0]["failure_reasons"])

    def test_manifest_path_symlink_is_rejected(self) -> None:
        """API callers must not use a linked manifest to change path base."""

        if not hasattr(os, "symlink"):
            self.skipTest("symlink unavailable")
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            target = root / "real"
            target.mkdir()
            linked = root / "linked"
            os.symlink(target, linked, target_is_directory=True)
            manifest = self._minimal_input_manifest(root, self._safe_report())
            with self.assertRaises(ValueError):
                build_input_manifest_report(manifest, manifest_path=linked / "inputs.json")

    def test_manifest_requires_source_and_artifact_sha(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report = self._safe_report()
            manifest = self._minimal_input_manifest(root, report)
            manifest["inputs"][0].pop("sha256")
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("source sha256 is required for a present source", assessed["inputs"][0]["failure_reasons"])
            artifact = root / "model.xml"
            artifact.write_text("model", encoding="utf-8")
            manifest = self._minimal_input_manifest(root, report)
            manifest["artifacts"] = [{"role": "model_xml", "path": str(artifact)}]
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("artifact sha256 is required for a present artifact", assessed["artifacts"][0]["failure_reasons"])

    def test_manifest_rejects_absence_status_on_present_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            artifact = root / "model.xml"
            artifact.write_text("model", encoding="utf-8")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            manifest = self._minimal_input_manifest(root, self._safe_report())
            manifest["artifacts"] = [{
                "role": "model_xml",
                "path": str(artifact),
                "sha256": digest,
                "absence_status": "NOT_VERIFIED",
            }]
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "absence_status contradicts present artifact",
                assessed["artifacts"][0]["failure_reasons"],
            )

    def test_manifest_rejects_absence_status_on_present_source(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report = self._safe_report()
            manifest = self._minimal_input_manifest(
                root, report, absence_status="NOT_VERIFIED",
            )
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "absence_status contradicts present source",
                assessed["inputs"][0]["failure_reasons"],
            )

    def test_manifest_rejects_ctest_skip_and_bad_case_consistency(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            report = self._safe_report(
                counts={"PASS": 1, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0},
                cases=[{"name": "one", "status": "PASS"}],
            )
            ctest = root / "Test.xml"
            ctest.write_text("<Site><Testing><Test Status=\"passed\"><Name>one</Name></Test></Testing></Site>", encoding="utf-8")
            manifest = self._minimal_input_manifest(root, report, kind="ctest", ctest_xml=str(ctest), skip_ctest=True)
            # ctest is an allowed input kind; skip_ctest is not.
            manifest["inputs"][0]["ctest_xml_sha256"] = hashlib.sha256(ctest.read_bytes()).hexdigest()
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("skip_ctest is not permitted in explicit admission mode", assessed["inputs"][0]["failure_reasons"])
            manifest["inputs"][0].pop("skip_ctest")
            report["cases"] = [{"name": "two", "status": "PASS"}]
            report_path = Path(manifest["inputs"][0]["path"])
            report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
            manifest["inputs"][0]["sha256"] = hashlib.sha256(report_path.read_bytes()).hexdigest()
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("report case names disagree with CTest XML", assessed["inputs"][0]["failure_reasons"])

    def test_manifest_ctest_xml_digest_cannot_be_self_attested_by_report(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            ctest = root / "Test.xml"
            ctest.write_text(
                "<Site><Testing><Test Status=\"passed\"><Name>one</Name></Test></Testing></Site>",
                encoding="utf-8",
            )
            report = self._safe_report(
                counts={"PASS": 1, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0},
                cases=[{"name": "one", "status": "PASS"}],
                ctest_xml_sha256=hashlib.sha256(ctest.read_bytes()).hexdigest(),
            )
            manifest = self._minimal_input_manifest(
                root, report, kind="ctest", ctest_xml=str(ctest)
            )
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn("CTest XML sha256 is required", assessed["inputs"][0]["failure_reasons"])

    def test_manifest_rejects_source_ctest_digest_mismatch(self) -> None:
        """A producer-side XML digest must agree with the actual XML bytes."""

        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            ctest = root / "Test.xml"
            ctest.write_text(
                "<Site><Testing><Test Status=\"passed\"><Name>one</Name></Test></Testing></Site>",
                encoding="utf-8",
            )
            report = self._safe_report(
                counts={"PASS": 1, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0},
                cases=[{"name": "one", "status": "PASS"}],
                ctest_xml_sha256="0" * 64,
            )
            manifest = self._minimal_input_manifest(
                root, report, kind="ctest", ctest_xml=str(ctest),
                ctest_xml_sha256=hashlib.sha256(ctest.read_bytes()).hexdigest(),
            )
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "source ctest_xml_sha256 does not match CTest XML bytes",
                assessed["inputs"][0]["failure_reasons"],
            )

    def test_manifest_rejects_liveness_exit_aliases_and_hardware_pass(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            for code in (-1, 130, 137):
                report = self._safe_report(
                    node_liveness={
                        "alive_before_sampling": True, "alive_during_sampling": True,
                        "alive_after_sampling": True, "exit_code_matches": True,
                        "expected_exit_code": code, "observed_exit_code": code,
                    }
                )
                manifest = self._minimal_input_manifest(root, report, kind="ros_e2e")
                assessed = build_input_manifest_report(manifest)
                self.assertNotEqual(assessed["software_candidate_status"], "READY_CANDIDATE")
            manifest = self._minimal_input_manifest(root, self._safe_report(), kind="hardware")
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")

    def test_manifest_artifact_role_map_and_duplicate_json_key_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            artifact = root / "model.xml"
            artifact.write_text("model", encoding="utf-8")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            report = self._safe_report(artifact_hashes={"model_xml": "0" * 64})
            manifest = self._minimal_input_manifest(root, report)
            manifest["artifacts"] = [{"role": "model_xml", "path": str(artifact), "sha256": digest}]
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertFalse(assessed["consistency"]["artifact_hash_consistent"])
            duplicate = root / "duplicate.json"
            duplicate.write_text('{"status":"FAIL","status":"PASS"}', encoding="utf-8")
            with self.assertRaises(ValueError):
                _load_observations(duplicate)

    def test_manifest_negative_evidence_requires_exact_contract(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            fixture = "fixture-data"
            fixture_sha = hashlib.sha256(fixture.encode()).hexdigest()
            report = self._safe_report(
                status="FAIL", report_status="FAIL", production_claim_rejected=True,
                negative_test=True, expected_failure=True,
                fixture_sha256=fixture_sha,
                exit_code=1,
                diagnostics={"errors": [{"code": "calibration_promotion"}], "warnings": []},
                synthetic=True, test_only=True, production_ready=False,
            )
            manifest = self._minimal_input_manifest(
                root, report, kind="evidence_bundle", negative_test=True,
                expected_failure=True, expected_diagnostic_code="calibration_promotion",
                expected_exit_code=1, fixture_sha256=fixture_sha,
            )
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "READY_CANDIDATE")
            report["status"] = "PASS"
            report_path = Path(manifest["inputs"][0]["path"])
            report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
            manifest["inputs"][0]["sha256"] = hashlib.sha256(report_path.read_bytes()).hexdigest()
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")

    def test_manifest_negative_evidence_aliases_and_warnings_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = Path(parent)
            fixture = "fixture-data"
            fixture_sha = hashlib.sha256(fixture.encode()).hexdigest()

            def make_report(**extra):
                report = self._safe_report(
                    status="FAIL", report_status="FAIL", production_claim_rejected=True,
                    fixture_sha256=fixture_sha, exit_code=1,
                    diagnostics={"errors": [{"code": "calibration_promotion"}], "warnings": []},
                    synthetic=True, test_only=True, production_ready=False,
                )
                report.update(extra)
                return report

            def make_manifest(report, **declaration):
                return self._minimal_input_manifest(
                    root, report, kind="evidence_bundle", negative_test=True,
                    expected_failure=True, expected_diagnostic_code="calibration_promotion",
                    expected_exit_code=1, fixture_sha256=fixture_sha, **declaration,
                )

            report = make_report()
            report["diagnostics"].pop("warnings")
            manifest = make_manifest(report)
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "negative evidence diagnostics.warnings must be an empty array",
                assessed["inputs"][0]["failure_reasons"],
            )

            report = make_report()
            manifest = make_manifest(report, expected_failure_exit_code=2)
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "negative evidence expected exit-code aliases disagree",
                assessed["inputs"][0]["failure_reasons"],
            )

            report = make_report(observed_exit_code=2)
            manifest = make_manifest(report)
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertIn(
                "negative evidence observed exit-code aliases disagree",
                assessed["inputs"][0]["failure_reasons"],
            )

            report = make_report(claims={"production_ready": True})
            manifest = make_manifest(report)
            assessed = build_input_manifest_report(manifest)
            self.assertEqual(assessed["software_candidate_status"], "BLOCKED")
            self.assertTrue(any(
                "unexpected unsafe claim" in reason
                for reason in assessed["inputs"][0]["failure_reasons"]
            ))


if __name__ == "__main__":
    unittest.main()

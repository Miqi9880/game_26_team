#!/usr/bin/env python3
"""CLI for the read-only model/profile and offline replay admission gate."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

if __package__ in {None, ""}:
    ROOT = Path(__file__).resolve().parents[2]
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from tools.auto_aim_qualification.offline_qualification import qualify_offline, render_markdown
else:
    from .offline_qualification import qualify_offline, render_markdown


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only model/profile and offline evidence qualification")
    parser.add_argument("--mode", choices=("evidence_only", "strict"), default="evidence_only")
    parser.add_argument("--model-profile", required=True)
    parser.add_argument("--model")
    parser.add_argument("--pnp-config", required=True)
    parser.add_argument("--input-csv")
    parser.add_argument("--metadata-json")
    parser.add_argument("--evidence-bundle", "--output-dir", dest="evidence_bundle")
    parser.add_argument("--manifest", "--verify-manifest", dest="manifest")
    parser.add_argument("--camera-intrinsic-report")
    parser.add_argument("--annotated-dir")
    parser.add_argument("--producer-command-file")
    parser.add_argument("--dataset-id")
    parser.add_argument("--commit")
    parser.add_argument("--run-id")
    parser.add_argument("--run-command")
    parser.add_argument("--model-sha256")
    parser.add_argument("--image-width", type=int)
    parser.add_argument("--image-height", type=int)
    parser.add_argument("--allow-test-only", "--allow-test-profile", action="store_true")
    parser.add_argument("--output-json", "--qualification-json", default="qualification.json")
    parser.add_argument("--output-markdown", "--qualification-markdown", default="qualification.md")
    return parser


def _same_or_alias(left: Path, right: Path) -> bool:
    """Return true for equal, symlink or hardlink paths without following writes."""

    try:
        return os.path.samefile(left, right)
    except (FileNotFoundError, OSError):
        try:
            return left.resolve(strict=False) == right.resolve(strict=False)
        except OSError:
            return os.path.abspath(left) == os.path.abspath(right)


def _validate_outputs(input_csv: str | None, json_path: str | None, markdown_path: str | None) -> None:
    paths = [Path(item) for item in (json_path, markdown_path) if item]
    if len(paths) == 2 and _same_or_alias(paths[0], paths[1]):
        raise OSError("qualification JSON and Markdown outputs must be distinct")
    if input_csv:
        source = Path(input_csv)
        for destination in paths:
            if _same_or_alias(source, destination):
                raise OSError("qualification output aliases the input CSV")
    for destination in paths:
        if destination.exists() and (destination.is_symlink() or destination.stat().st_nlink > 1):
            raise OSError("qualification output must not be a symlink or hardlink")
        current = destination.parent
        while current != current.parent:
            if current.exists() and current.is_symlink():
                raise OSError("qualification output parent must not be a symlink")
            current = current.parent


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    outputs_safe = True
    try:
        _validate_outputs(args.input_csv, args.output_json, args.output_markdown)
        report = qualify_offline(
            model_profile=args.model_profile,
            model=args.model,
            pnp_config=args.pnp_config,
            mode=args.mode,
            allow_test_only=args.allow_test_only,
            input_csv=args.input_csv,
            metadata_json=args.metadata_json,
            evidence_bundle=args.evidence_bundle,
            manifest=args.manifest,
            camera_intrinsic_report=args.camera_intrinsic_report,
            annotated_dir=args.annotated_dir,
            producer_command_file=args.producer_command_file,
            expected_image_size=(args.image_width, args.image_height) if args.image_width and args.image_height else None,
            metadata={
                key: value for key, value in {
                    "dataset_id": args.dataset_id,
                    "commit": args.commit,
                    "run_id": args.run_id,
                    "run_command": args.run_command,
                    "model_sha256": args.model_sha256,
                }.items() if value not in (None, "")
            },
        )
    except Exception as exc:  # malformed input must still yield a report
        if isinstance(exc, OSError) and str(exc).startswith("qualification output"):
            outputs_safe = False
        report = {
            "schema_version": 1,
            "status": "FAIL",
            "mode": args.mode,
            "production_ready": False,
            "hardware_validation": False,
            "gimbal_closed_loop_validated": False,
            "firing_validated": False,
            "real_hit_rate_computed": False,
            "diagnostics": {"errors": [{"status": "FAIL", "code": "qualification_exception", "message": type(exc).__name__}], "warnings": []},
            "safety_boundary": {"serial_enabled": False, "dry_run": True, "allow_fire": False, "fire_command": 0, "yaw_vel": 0, "pitch_vel": 0, "yaw_acc": 0, "pitch_acc": 0},
        }
    if outputs_safe:
        try:
            for path, payload in ((args.output_json, json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"), (args.output_markdown, render_markdown(report))):
                if not path:
                    continue
                destination = Path(path)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(payload, encoding="utf-8")
        except OSError as exc:
            report.setdefault("diagnostics", {}).setdefault("errors", []).append({"status": "FAIL", "code": "qualification_output_error", "message": type(exc).__name__})
            report["status"] = "FAIL"
    print(f"status={report.get('status', 'FAIL')}")
    return {"PASS": 0, "WARN": 2, "FAIL": 1}.get(str(report.get("status")), 1)


if __name__ == "__main__":
    raise SystemExit(main())

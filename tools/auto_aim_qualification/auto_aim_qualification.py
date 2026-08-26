#!/usr/bin/env python3
"""CLI for the read-only model/profile and offline replay admission gate."""

from __future__ import annotations

import argparse
import json
import os
import stat
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
    parser.add_argument("--model-bin")
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
    parser.add_argument("--model-bin-sha256")
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
    except FileNotFoundError:
        pass
    except (OSError, ValueError) as exc:
        raise OSError("qualification output/input paths cannot be inspected") from exc
    try:
        return left.resolve(strict=False) == right.resolve(strict=False)
    except (OSError, RuntimeError) as exc:
        raise OSError("qualification output/input paths cannot be resolved") from exc


def _path_is_within(path: Path, directory: Path) -> bool:
    """Return whether a path is equal to or below a directory after resolution."""

    try:
        path.resolve(strict=False).relative_to(directory.resolve(strict=False))
    except ValueError:
        return False
    except (OSError, RuntimeError) as exc:
        raise OSError("qualification output/input paths cannot be resolved") from exc
    return True


def _validate_outputs(
    input_csv: str | None,
    json_path: str | None,
    markdown_path: str | None,
    *,
    model_profile: str | None = None,
    pnp_config: str | None = None,
    model: str | None = None,
    model_bin: str | None = None,
    metadata_json: str | None = None,
    evidence_bundle: str | None = None,
    manifest: str | None = None,
    camera_intrinsic_report: str | None = None,
    annotated_dir: str | None = None,
    producer_command_file: str | None = None,
) -> None:
    paths = [Path(item) for item in (json_path, markdown_path) if item]
    if len(paths) == 2 and _same_or_alias(paths[0], paths[1]):
        raise OSError("qualification output aliases another qualification output")

    # Every file below is read-only evidence/configuration.  A report path
    # must never replace it, even when the path is supplied through a symlink
    # or hardlink alias.  Check all outputs before qualification starts so a
    # safe-looking first report cannot be written before a later collision is
    # discovered.
    protected_files = [
        Path(item)
        for item in (
            model_profile,
            pnp_config,
            model,
            model_bin,
            input_csv,
            metadata_json,
            manifest,
            camera_intrinsic_report,
            producer_command_file,
        )
        if item
    ]
    for destination in paths:
        for source in protected_files:
            if _same_or_alias(source, destination):
                raise OSError("qualification output aliases a read-only input")

        # Directory inputs are protected by containment, not only exact path
        # equality.  This covers an output such as annotated/report.json and
        # also reserves the evidence bundle root, whose manifest would become
        # stale or gain an undeclared qualification report otherwise.
        protected_directories = [item for item in (annotated_dir, evidence_bundle) if item]
        if manifest:
            # A manifest's parent is the evidence bundle root when callers
            # provide only --manifest.  Adding it here prevents a new
            # qualification report from becoming an undeclared bundle file.
            protected_directories.append(str(Path(manifest).parent))
        for directory in protected_directories:
            if directory and _path_is_within(destination, Path(directory)):
                raise OSError("qualification output is inside a read-only/output bundle directory")

    for destination in paths:
        _validate_output_destination(destination)


def _validate_output_destination(destination: Path) -> None:
    """Reject output paths that could redirect a write outside their parent.

    ``Path.exists()`` follows symlinks and returns ``False`` for a dangling
    link.  Checking the path with ``lstat`` instead makes both live and
    dangling symlinks visible before any parent directory is created or file
    is opened.  Existing hardlinks are rejected as well, since truncating one
    would mutate every name for the inode.
    """

    destination = Path(destination)
    try:
        info = destination.lstat()
    except FileNotFoundError:
        info = None
    except OSError as exc:
        raise OSError(f"qualification output cannot be inspected: {destination}") from exc
    if info is not None:
        if stat.S_ISLNK(info.st_mode):
            raise OSError("qualification output must not be a symlink")
        if not stat.S_ISREG(info.st_mode):
            raise OSError("qualification output must be a regular file")
        if info.st_nlink > 1:
            raise OSError("qualification output must not be a hardlink")

    # Walk lexical parent components with lstat.  In particular, lstat sees a
    # dangling parent symlink that ``exists()`` would incorrectly hide.
    current = destination.parent
    while True:
        try:
            parent_info = current.lstat()
        except FileNotFoundError:
            parent_info = None
        except OSError as exc:
            raise OSError(f"qualification output parent cannot be inspected: {current}") from exc
        if parent_info is not None:
            if stat.S_ISLNK(parent_info.st_mode):
                raise OSError("qualification output parent must not be a symlink")
            if not stat.S_ISDIR(parent_info.st_mode):
                raise OSError("qualification output parent must be a directory")
        if current == current.parent:
            break
        current = current.parent


def _ensure_output_parent(destination: Path) -> None:
    """Create missing output parents one component at a time.

    ``Path.mkdir(parents=True)`` may traverse a parent symlink that appears
    after the initial validation.  Creating each lexical component separately
    lets us reject a component immediately after a concurrent ``EEXIST`` or
    other replacement, before the report file is opened.
    """

    parent = Path(destination).parent.absolute()
    current = Path(parent.anchor) if parent.anchor else Path.cwd()
    for part in parent.parts:
        if part == parent.anchor:
            continue
        current = current / part
        try:
            info = current.lstat()
        except FileNotFoundError:
            try:
                current.mkdir()
            except FileExistsError:
                # A concurrent creator may have won the race.  Inspect the
                # resulting entry below rather than trusting its type.
                pass
            try:
                info = current.lstat()
            except OSError as exc:
                raise OSError(f"qualification output parent cannot be inspected: {current}") from exc
        except OSError as exc:
            raise OSError(f"qualification output parent cannot be inspected: {current}") from exc
        if stat.S_ISLNK(info.st_mode):
            raise OSError("qualification output parent must not be a symlink")
        if not stat.S_ISDIR(info.st_mode):
            raise OSError("qualification output parent must be a directory")


def _write_output_text(destination: Path, payload: str) -> None:
    """Write one report without following a destination symlink.

    The destination is validated immediately before opening it and, on
    platforms that support it, ``O_NOFOLLOW`` closes the validation/write
    race for a symlink swapped in between those operations.
    """

    destination = Path(destination)
    # Validate before creating any missing parent components.  Calling
    # ``mkdir(parents=True)`` first could follow a dangling symlink in an
    # intermediate component and create directories in an external tree.
    _validate_output_destination(destination)
    _ensure_output_parent(destination)
    _validate_output_destination(destination)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    if nofollow:
        flags |= nofollow
    try:
        descriptor = os.open(destination, flags, 0o600)
    except OSError as exc:
        raise OSError(f"qualification output could not be opened: {destination}") from exc
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
    except Exception:
        # ``fdopen`` owns the descriptor after a successful call.  If it fails
        # before ownership is transferred, avoid leaking the descriptor.
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    outputs_safe = True
    try:
        _validate_outputs(
            args.input_csv,
            args.output_json,
            args.output_markdown,
            model_profile=args.model_profile,
            pnp_config=args.pnp_config,
            model=args.model,
            model_bin=args.model_bin,
            metadata_json=args.metadata_json,
            evidence_bundle=args.evidence_bundle,
            manifest=args.manifest,
            camera_intrinsic_report=args.camera_intrinsic_report,
            annotated_dir=args.annotated_dir,
            producer_command_file=args.producer_command_file,
        )
        report = qualify_offline(
            model_profile=args.model_profile,
            model=args.model,
            model_bin=args.model_bin,
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
                **{
                    key: value for key, value in {
                        "dataset_id": args.dataset_id,
                        "commit": args.commit,
                        "run_id": args.run_id,
                        "run_command": args.run_command,
                    }.items() if value not in (None, "")
                },
                # An explicitly supplied empty hash is still a declaration
                # and must fail closed; do not let the generic optional-field
                # filtering silently discard it.
                **({"model_sha256": args.model_sha256} if args.model_sha256 is not None else {}),
                **({"model_bin_sha256": args.model_bin_sha256} if args.model_bin_sha256 is not None else {}),
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
                _write_output_text(destination, payload)
        except OSError as exc:
            report.setdefault("diagnostics", {}).setdefault("errors", []).append({"status": "FAIL", "code": "qualification_output_error", "message": type(exc).__name__})
            report["status"] = "FAIL"
    print(f"status={report.get('status', 'FAIL')}")
    return {"PASS": 0, "WARN": 2, "FAIL": 1}.get(str(report.get("status")), 1)


if __name__ == "__main__":
    raise SystemExit(main())

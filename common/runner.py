"""Reproducible execution and mandatory validation of Min-Min submissions."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import shutil
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

from . import common as c
from .metrics import evaluate_metrics
from .slicer import build_tasks

PACKAGE = Path(__file__).resolve().parents[1]


def sha256(path):
    """Hash artifacts incrementally, including large binary STL inputs."""
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def replace_file(temporary, destination):
    """Retry transient Windows sharing violations without truncating old files."""
    for attempt in range(6):
        try:
            os.replace(temporary, destination)
            return
        except PermissionError:
            if attempt == 5:
                raise
            time.sleep(min(0.1 * 2**attempt, 1.0))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
        replace_file(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def publish(source, destination):
    """Publish only the exact trajectory accepted by the full Validator."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=destination.name + ".", dir=destination.parent)
    os.close(fd)
    try:
        shutil.copyfile(source, temporary)
        replace_file(temporary, destination)
    finally:
        Path(temporary).unlink(missing_ok=True)


def validate(inputs, output, expected):
    from waam_validator.pipeline import run_validation
    from waam_validator.reporting.writers import CORE_RESULT_FILES

    started = time.perf_counter()
    record = dict(status="ERROR", failure_reasons=[], makespan_matches=False)
    try:
        result = run_validation(inputs, output)
        actual = result.schedule.makespan_s
        matches = math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6)
        reasons = list(result.failure_reasons)
        if not matches:
            reasons.append(f"MAKESPAN_MISMATCH: scheduler={expected}, validator={actual}")
        complete = all((output / name).is_file() for name in CORE_RESULT_FILES)
        if not complete:
            reasons.append("INCOMPLETE_VALIDATOR_REPORT")
        record.update(
            status="PASS" if result.status == "PASS" and matches and complete else "FAIL",
            validator_status=result.status,
            expected_makespan_s=expected,
            validator_makespan_s=actual,
            makespan_matches=matches,
            failure_reasons=reasons,
            warnings=[f"{issue.code}: {issue.message}" for issue in result.warnings],
        )
    except Exception as exc:
        record["failure_reasons"] = [f"{type(exc).__name__}: {exc}"]
    record["validation_time_s"] = time.perf_counter() - started
    record["report_directory"] = "report"
    return record


def environment():
    packages = {}
    for name in ("waam-validator", "numpy", "shapely", "trimesh", "PyYAML", "pydantic"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = "not-installed"
    return dict(python=sys.version, platform=platform.platform(), packages=packages)


def execute_case(schedule, source, folder, published, source_hashes):
    """Generate from immutable input copies; never load previous schedules."""
    started = time.perf_counter()
    folder.mkdir(parents=True, exist_ok=False)
    inputs = folder / "input"
    inputs.mkdir()
    manifest = dict(
        algorithm="min_min",
        status="RUNNING",
        source_sha256=source_hashes,
        environment=environment(),
        input_sha256={},
        artifact_sha256={},
        requested_trajectory=str(published.resolve()),
    )
    write_json(folder / "manifest.json", manifest)
    try:
        for name in ("config.yaml", "target.stl"):
            shutil.copyfile(source / name, inputs / name)
            manifest["input_sha256"][name] = sha256(inputs / name)
        write_json(folder / "manifest.json", manifest)
        c.load_config(inputs / "config.yaml")
        stage = time.perf_counter()
        print(f"[{folder.name}] Slicing {source / 'target.stl'}", flush=True)
        tasks = build_tasks(inputs / "target.stl", c.CONFIG)
        generation_time = time.perf_counter() - stage
        print(f"[{folder.name}] Min-Min: {len(tasks)} tasks", flush=True)
        stage = time.perf_counter()

        def progress(layer, done, total):
            print(
                f"[{folder.name}] Layer {layer} | {done}/{total} tasks | "
                f"{time.perf_counter() - stage:.1f} s",
                flush=True,
            )

        states, assignments = schedule(tasks, progress=progress)
        computation_time = time.perf_counter() - stage
        metrics = evaluate_metrics(states, assignments, computation_time)
        metrics.update(tasks=len(tasks), task_generation_time_s=generation_time)
        trajectory = inputs / "trajectory.csv"
        c.write_trajectory(states, trajectory)
        c.write_assignments(assignments, folder / "assignments.csv")
        print(
            f"[{folder.name}] Validating | Makespan {metrics['makespan_s']:.6f} s | "
            f"Computation {computation_time:.3f} s",
            flush=True,
        )
        validation = validate(inputs, folder / "report", metrics["makespan_s"])
        write_json(folder / "validation.json", validation)
        metrics.update(
            trajectory_validation=validation["status"],
            validation_time_s=validation["validation_time_s"],
        )
        write_json(folder / "metrics.json", metrics)
        manifest["artifact_sha256"] = {
            str(path.relative_to(folder).as_posix()): sha256(path)
            for path in sorted(folder.rglob("*"))
            if path.is_file() and path.name != "manifest.json"
        }
        if validation["status"] != "PASS":
            raise ValueError(
                "Full Validator rejected the trajectory: "
                + " | ".join(validation["failure_reasons"])
            )
        # A failed attempt leaves any previously published, validated CSV intact.
        publish(trajectory, published)
        manifest.update(status="PASS", published_sha256=sha256(published))
        print(f"[{folder.name}] PASS | {published}", flush=True)
    except Exception as exc:
        manifest.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        manifest["total_time_s"] = time.perf_counter() - started
        write_json(folder / "manifest.json", manifest)


def run(schedule, argv=None):
    parser = argparse.ArgumentParser(description="Generate and validate Min-Min WAAM trajectories.")
    parser.add_argument(
        "case",
        nargs="?",
        type=int,
        choices=range(1, 7),
        help="Benchmark case number; default: all six cases",
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        help="Directory containing 01/config.yaml, 01/target.stl, ..., 06/",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        help="One arbitrary directory containing config.yaml and target.stl",
    )
    parser.add_argument("--output-dir", type=Path, default=PACKAGE / "trajectory")
    parser.add_argument("--results-dir", type=Path, default=PACKAGE / "results")
    args = parser.parse_args(argv)
    if args.input_dir is not None and (args.case is not None or args.input_root is not None):
        parser.error("--input-dir cannot be combined with a case number or --input-root")
    if args.input_dir is not None:
        cases = [("custom", args.input_dir.resolve())]
    else:
        root = args.input_root
        if root is None:
            root = PACKAGE / "inputs"
            if not root.is_dir():
                root = PACKAGE.parent / "tests"
        cases = [
            (f"{case:02d}", root.resolve() / f"{case:02d}")
            for case in ([args.case] if args.case is not None else range(1, 7))
        ]
    for _, source in cases:
        for name in ("config.yaml", "target.stl"):
            if not (source / name).is_file():
                parser.error(f"Missing input: {source / name}. Set --input-root or --input-dir.")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    session = args.results_dir.resolve() / stamp
    session.mkdir(parents=True, exist_ok=False)
    sources = [PACKAGE / "Min-Min.py", *sorted((PACKAGE / "common").glob("*.py"))]
    hashes = {p.relative_to(PACKAGE).as_posix(): sha256(p) for p in sources}
    write_json(
        session / "run.json",
        dict(source_sha256=hashes, environment=environment(), cases=[label for label, _ in cases]),
    )
    print(f"Run records: {session}", flush=True)
    failed = False
    for label, source in cases:
        try:
            execute_case(
                schedule,
                source,
                session / f"case_{label}",
                args.output_dir.resolve() / f"trajectory_{label}.csv",
                hashes,
            )
        except Exception as exc:
            failed = True
            print(f"[{label}] ERROR: {exc}", file=sys.stderr, flush=True)
    return 1 if failed else 0

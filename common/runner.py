"""Reproducible execution and mandatory validation of Min-Min submissions."""

from __future__ import annotations

import gzip
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
from pathlib import Path

from . import common as c
from .checkpoint import digest
from .metrics import evaluate_metrics, parallelism_diagnostics
from .provided_paths import build_tasks, verify_deposition

PACKAGE = Path(__file__).resolve().parents[1]


def format_duration(seconds):
    """Display elapsed seconds as DD:HH:MM:SS, rounded to the nearest second."""
    total = math.floor(seconds + 0.5)
    days, remainder = divmod(total, 86400)
    hours, remainder = divmod(remainder, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{days:02d}:{hours:02d}:{minutes:02d}:{seconds:02d}"


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
    """Publish a file from the input bundle accepted by the full Validator."""
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


def execute_case(schedule, source, folder, published, source_hashes, checkpoint_source=None):
    """Allocate immutable supplied paths, then validate before publishing."""
    started = time.perf_counter()
    folder.mkdir(parents=True, exist_ok=False)
    inputs = folder / "input"
    inputs.mkdir()
    manifest = dict(
        algorithm="min_min",
        travel_policy="free_initial_optional_egress_timed_waits",
        precedence_policy="local_support",
        selection_policy="lower_bound_pruning",
        task_source="generated_from_input_stl",
        status="RUNNING",
        source_sha256=source_hashes,
        environment=environment(),
        input_sha256={},
        artifact_sha256={},
        requested_trajectory=str((published / "trajectory.csv").resolve()),
    )
    write_json(folder / "manifest.json", manifest)
    try:
        path_name = ("deposition_paths.json.gz" if (source / "deposition_paths.json.gz").exists()
                     else "deposition_paths.json")
        for name in ("config.yaml", "target.stl", path_name):
            shutil.copyfile(source / name, inputs / name)
            manifest["input_sha256"][name] = sha256(inputs / name)
        write_json(folder / "manifest.json", manifest)
        c.load_config(inputs / "config.yaml")
        stage = time.perf_counter()
        print(f"[{folder.name}] Loading fixed deposition paths", flush=True)
        tasks = build_tasks(inputs / "target.stl", c.CONFIG)
        opener = gzip.open if path_name.endswith(".gz") else open
        with opener(inputs / path_name, "rt", encoding="utf-8") as stream:
            manifest["path_provenance"] = json.load(stream)["provenance"]
        generation_time = time.perf_counter() - stage
        print(f"[{folder.name}] Min-Min: {len(tasks)} tasks", flush=True)
        stage = time.perf_counter()
        resume_state, prior_computation = None, 0.0
        if checkpoint_source is not None:
            saved = json.loads(checkpoint_source.read_text(encoding="utf-8"))
            if (saved.get("schema") != 1 or saved.get("source_sha256") != source_hashes
                    or saved.get("input_sha256") != manifest["input_sha256"]
                    or saved.get("state_sha256") != digest(saved["state"])):
                raise ValueError("Checkpoint code, input or state checksum mismatch")
            prior_computation = float(saved["computation_time_s"])
            if not math.isfinite(prior_computation) or prior_computation < 0:
                raise ValueError("Invalid checkpoint computation time")
            resume_state = saved["state"]
            manifest["resumed_from"] = str(checkpoint_source.resolve())
            manifest["prior_computation_time_s"] = prior_computation

        def progress(layer, done, total):
            print(
                f"[{folder.name}] Layer {layer} | {done}/{total} tasks | "
                f"Elapsed {format_duration(prior_computation + time.perf_counter() - stage)}",
                flush=True,
            )

        support = {}
        def save_state(state):
            write_json(folder / "schedule_checkpoint.json", dict(
                schema=1, source_sha256=source_hashes, input_sha256=manifest["input_sha256"],
                state=state, state_sha256=digest(state),
                computation_time_s=prior_computation + time.perf_counter() - stage))

        states, assignments = schedule(tasks, progress=progress, diagnostics=support,
                                       resume_state=resume_state, save_state=save_state)
        computation_time = prior_computation + time.perf_counter() - stage
        write_json(folder / "support_dependencies.json", support)
        selection = support["selection"]
        print(f"[{folder.name}] Candidate evaluations {selection['evaluated_pairs']} / "
              f"{selection['candidate_pairs']} | Pruned {selection['pruned_pairs']}", flush=True)
        if not selection.get("prefix_counts_included", True):
            print(f"[{folder.name}] Candidate counts cover the resumed portion only", flush=True)
        print(f"[{folder.name}] Support edges {support['dependency_count']} | "
              f"Fallback tasks {support['fallback_task_count']} | "
              f"Early upper-layer tasks {support['tasks_started_before_previous_layer_finished']}",
              flush=True)
        identity = verify_deposition(states, tasks)
        write_json(folder / "path_identity.json", identity)
        metrics = evaluate_metrics(states, assignments, computation_time)
        metrics.update(tasks=len(tasks), task_loading_time_s=generation_time,
                       deposition_length_mm=identity["total_deposition_length_mm"],
                       source_kind=manifest["path_provenance"]["source_kind"])
        metrics.update(support_dependency_count=support["dependency_count"],
                       support_fallback_task_count=support["fallback_task_count"],
                       early_upper_layer_task_count=support["tasks_started_before_previous_layer_finished"])
        metrics.update(candidate_pairs=selection["candidate_pairs"],
                       evaluated_pairs=selection["evaluated_pairs"], pruned_pairs=selection["pruned_pairs"],
                       candidate_counts_include_prefix=selection.get("prefix_counts_included", True))
        connections = [a for a in assignments if a["transition_from_task"] not in ("home", "initial")]
        metrics.update(
            straight_transition_count=sum(a["approach_mode"] == "straight" for a in connections),
            safe_height_transition_count=sum(a["approach_mode"] == "safe_height" for a in connections),
            total_approach_robot_time_s=sum(a["approach_time_s"] for a in assignments),
            total_return_robot_time_s=sum(a["return_home_time_s"] for a in assignments),
            initial_at_task_robot_count=sum(
                a["transition_from_task"] == "initial" for a in assignments),
            terminal_at_task_robot_count=sum(
                max((a for a in assignments if a["robot_id"] == rid),
                    key=lambda a: a["job_start_s"], default={}).get("return_home_time_s") == 0
                for rid in c.ROBOTS),
            optional_return_count=sum(a["return_home_time_s"] > 0 for a in assignments),
        )
        trajectory = inputs / "trajectory.csv"
        c.write_trajectory(states, trajectory)
        c.write_assignments(assignments, folder / "assignments.csv")
        write_json(folder / "parallelism.json", parallelism_diagnostics(assignments, metrics["makespan_s"]))
        print(
            f"[{folder.name}] Validating | Makespan {format_duration(metrics['makespan_s'])} | "
            f"Computation Time {format_duration(computation_time)}",
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
        # Publish all three Validator inputs together in the case directory.
        # A validation rejection leaves the previously published bundle intact.
        for name in ("config.yaml", "target.stl", "trajectory.csv"):
            publish(inputs / name, published / name)
        manifest.update(
            status="PASS",
            published_sha256=sha256(published / "trajectory.csv"),
            published_input_sha256={
                name: sha256(published / name)
                for name in ("config.yaml", "target.stl", "trajectory.csv")
            },
        )
        print(f"[{folder.name}] PASS | {published}", flush=True)
    except Exception as exc:
        manifest.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        manifest["total_time_s"] = time.perf_counter() - started
        write_json(folder / "manifest.json", manifest)


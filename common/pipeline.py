"""STL-to-trajectory pipeline with geometry screening and certified publication.

No benchmark identifiers, historical trajectories or cached solutions are used.
The default shape budget is a policy, not a modification of Validator settings.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import shutil
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter

from waam_validator.shape.target import (
    determine_evaluation_layers,
    load_target_mesh,
    slice_target_layers,
)

from . import common as c
from . import path_budget, path_generation
from . import runner as r
from .checkpoint import digest
from .min_min import schedule

PACKAGE = Path(__file__).resolve().parents[1]


def source_hashes():
    return {p.relative_to(PACKAGE).as_posix(): r.sha256(p)
            for p in [PACKAGE / "Min-Min.py", *sorted((PACKAGE / "common").glob("*.py"))]}


def export_paths(folder, source, tasks, settings):
    folder.mkdir(parents=True)
    for name in ("config.yaml", "target.stl"):
        shutil.copyfile(source / name, folder / name)
    rows = [dict(task_id=t.task_id, layer=t.layer, points_mm=t.points) for t in tasks]
    provenance = dict(source_kind="generated_from_input_stl", task_count=len(tasks),
        total_deposition_length_mm=sum(t.length for t in tasks), settings=settings,
        input_sha256={name: r.sha256(folder / name) for name in ("config.yaml", "target.stl")})
    with gzip.open(folder / "deposition_paths.json.gz", "wt", encoding="utf-8") as stream:
        json.dump(dict(schema_version=1, tasks=rows, tasks_sha256=digest(rows),
                       provenance=provenance), stream, allow_nan=False)
    r.write_json(folder / "path_source.json", provenance)


def generate(session, settings):
    started = perf_counter()
    source = session / "input"
    c.load_config(source / "config.yaml")
    config = c.CONFIG
    floor = max(settings["iou_floor"], config.shape_validation.minimum_overall_iou,
                config.shape_validation.minimum_layer_iou)
    required = min(1., floor + settings["screening_margin"])
    mesh = load_target_mesh(source / "target.stl", config)
    indices = determine_evaluation_layers(mesh, {}, config)
    if not indices:
        raise ValueError("Target has no printable layers")
    targets = slice_target_layers(mesh, indices, config)
    original, trimmed, base_geometry, trimmed_geometry, decisions = [], [], {}, {}, []
    for n, (layer, target) in enumerate(sorted(targets.items()), 1):
        # Empty slices in disconnected vertical geometry are not printable jobs.
        if target.area <= config.shape_validation.area_epsilon_mm2:
            continue
        quality = max(.97, required)
        relaxed = False
        try:
            tasks, geometry, decision = path_generation.select_layer(
                target, layer, config, iou_floor=quality,
                max_length=settings["max_task_length_mm"])
        except ValueError:
            if quality <= required:
                raise
            tasks, geometry, decision = path_generation.select_layer(
                target, layer, config, iou_floor=required,
                max_length=settings["max_task_length_mm"])
            relaxed = True
        original.extend(tasks)
        base_geometry[layer] = geometry
        kept, kept_geometry = tasks, geometry
        reduction = dict(status="DISABLED")
        if settings["trim"]:
            try:
                kept, kept_geometry, reduction = path_budget.select_layer(
                    tasks, target, config, floor, margin=settings["screening_margin"])
                reduction["status"] = "SCREENED"
            except ValueError as exc:
                # Preserve the already screened base paths on a difficult layer.
                reduction = dict(status="BASE_PATHS_RETAINED", reason=str(exc))
        trimmed.extend(kept)
        trimmed_geometry[layer] = kept_geometry
        decisions.append(dict(layer=layer, generation=decision, trim=reduction,
                              relaxed_generation_floor=relaxed))
        if n % 10 == 0 or n == len(targets):
            print(f"[paths] {n}/{len(targets)} layers | {len(trimmed)} tasks | "
                  f"Elapsed {r.format_duration(perf_counter()-started)}", flush=True)
            r.write_json(session / "generation_progress.json", dict(
                layers_done=n, total_layers=len(targets), task_count=len(trimmed),
                elapsed_s=perf_counter()-started))
    if not original:
        raise ValueError("No printable paths were generated")
    base_shape, base_layers = path_generation.compute_shape_metrics(base_geometry, targets, config)
    shape, layers = path_generation.compute_shape_metrics(trimmed_geometry, targets, config)
    if not base_shape.passed or not shape.passed or shape.iou < required:
        raise ValueError("Complete target failed the official geometry screen")
    effective = dict(settings, effective_iou_floor=floor, screened_iou_floor=required)
    export_paths(session / "prepared/base", source, original, effective)
    changed = any(a.points != b.points for a, b in zip(original, trimmed))
    variants = ["base"]
    if settings["trim"] and changed:
        export_paths(session / "prepared/trimmed", source, trimmed, effective)
        variants.insert(0, "trimmed")
    r.write_json(session / "layer_decisions.json", decisions)
    r.write_json(session / "geometry_screen.json", dict(status="GEOMETRY_ONLY_PASS",
        base=dict(shape=asdict(base_shape), layers=[asdict(v) for v in base_layers]),
        selected=dict(shape=asdict(shape), layers=[asdict(v) for v in layers])))
    elapsed = perf_counter() - started
    report = dict(status="PREPARED", settings=effective, variants=variants,
        generation_time_s=elapsed, layer_count=len(targets), task_count=len(trimmed),
        base_length_mm=sum(t.length for t in original),
        selected_length_mm=sum(t.length for t in trimmed), overall_iou=shape.iou,
        min_layer_iou=min(v.iou for v in layers),
        prepared_sha256={v.relative_to(session).as_posix(): r.sha256(v)
                         for v in (session / "prepared").rglob("*") if v.is_file()})
    r.write_json(session / "generation.json", report)
    print(f"[paths] Geometry only | IoU {shape.iou:.3%} | {len(trimmed)} tasks | "
          f"Generation {r.format_duration(elapsed)}", flush=True)
    return report


def verify_session(session, hashes):
    record = json.loads((session / "run.json").read_text(encoding="utf-8"))
    if record["source_sha256"] != hashes or record["environment"] != r.environment():
        raise ValueError("Code/environment changed; start a new session")
    for name, expected in record["input_sha256"].items():
        if r.sha256(session / "input" / name) != expected:
            raise ValueError(f"Input changed: {name}")
    report = json.loads((session / "generation.json").read_text(encoding="utf-8"))
    if report["status"] != "PREPARED":
        raise ValueError("No complete prepared generation to resume")
    for name, expected in report["prepared_sha256"].items():
        if r.sha256(session / name) != expected:
            raise ValueError(f"Prepared path input changed: {name}")
    return report


def verify_pass(folder):
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    validation = json.loads((folder / "validation.json").read_text(encoding="utf-8"))
    if manifest["status"] != "PASS" or validation["status"] != "PASS":
        raise ValueError("Full Validator did not pass")
    required = {"input/config.yaml", "input/target.stl", "input/trajectory.csv",
                "validation.json", "metrics.json"}
    if not required.issubset(manifest["artifact_sha256"]):
        raise ValueError("Missing certified artifacts")
    for name, expected in manifest["artifact_sha256"].items():
        if r.sha256(folder / name) != expected:
            raise ValueError(f"Certified artifact changed: {name}")
    return json.loads((folder / "metrics.json").read_text(encoding="utf-8"))


def allocate(session, generation, hashes):
    rows = []
    for variant in generation["variants"]:
        root = session / "allocation" / variant
        previous = sorted(root.glob("attempt_*/manifest.json"))
        prior = json.loads(previous[-1].read_text()) if previous else {}
        checkpoint = None
        if prior.get("status") in ("PASS", "ERROR"):
            folder = previous[-1].parent
            expected_inputs = {name: r.sha256(session / "prepared" / variant / name)
                               for name in ("config.yaml", "target.stl", "deposition_paths.json.gz")}
            if prior.get("source_sha256") != hashes or prior.get("input_sha256") != expected_inputs:
                raise ValueError("Previous allocation does not match prepared inputs/code")
        else:
            if prior.get("status") == "RUNNING":
                path = previous[-1].parent / "schedule_checkpoint.json"
                if path.exists():
                    checkpoint = path
            folder = root / f"attempt_{len(previous)+1:03d}"
            started = perf_counter()
            try:
                r.execute_case(schedule, session / "prepared" / variant, folder,
                               session / "trajectory", hashes, checkpoint)
            except Exception as exc:
                print(f"[{variant}] ERROR | {type(exc).__name__}: {exc}", flush=True)
            finally:
                # Includes the resumed prefix when present. Validator time is
                # separated below; overhead and failed work remain visible.
                manifest = json.loads((folder / "manifest.json").read_text())
                r.write_json(folder / "attempt_timing.json", dict(
                    wall_time_s=perf_counter()-started + manifest.get("prior_computation_time_s", 0.)))
        manifest = json.loads((folder / "manifest.json").read_text())
        passed = manifest["status"] == "PASS"
        metric_file = folder / "metrics.json"
        metrics = json.loads(metric_file.read_text()) if metric_file.exists() else {}
        if passed:
            metrics = verify_pass(folder)
        timing = json.loads((folder / "attempt_timing.json").read_text())
        validation_time = metrics.get("validation_time_s", 0.)
        rows.append(dict(variant=variant, status="PASS" if passed else "ERROR",
            result_directory=str(folder), makespan_s=metrics.get("makespan_s") if passed else None,
            allocation_time_s=metrics.get("computation_time_s"),
            attempt_wall_time_s=timing["wall_time_s"], validation_time_s=validation_time,
            error=manifest.get("error")))
        r.write_json(session / "attempts.json", rows)
        if passed:
            # Successful reduced paths are accepted immediately. Untrimmed
            # allocation is a failure fallback, not a second optimization trial.
            for name in ("config.yaml", "target.stl", "trajectory.csv"):
                r.publish(folder / "input" / name, session / "trajectory" / name)
            computation = generation["generation_time_s"] + sum(
                max(0., row["attempt_wall_time_s"] - row["validation_time_s"]) for row in rows)
            result = dict(status="PASS", variant=variant, makespan_s=metrics["makespan_s"],
                computation_time_s=computation, generation_time_s=generation["generation_time_s"],
                allocation_time_s=metrics["computation_time_s"],
                validation_time_s=sum(row["validation_time_s"] for row in rows),
                result_directory=str(folder), validator_input_directory=str(session / "trajectory"),
                note="Computation includes generation, failed fallback work, allocation and export; excludes Validator.")
            r.write_json(session / "summary.json", result)
            print(f"PASS | Makespan {r.format_duration(result['makespan_s'])} | "
                  f"Total Computation Time {r.format_duration(computation)}", flush=True)
            print(f"Validator inputs: {session / 'trajectory'}", flush=True)
            return 0
        if variant != generation["variants"][-1]:
            print("[fallback] Retrying with untrimmed generated paths", flush=True)
    r.write_json(session / "summary.json", dict(status="ERROR", makespan_s=None,
        error="No candidate passed full validation", attempts=rows))
    return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate paths from STL, allocate with Min-Min, and validate.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input-dir", type=Path, help="Directory with config.yaml and target.stl")
    source.add_argument("--resume-session", type=Path, help="Reuse prepared paths and verified results/checkpoints")
    parser.add_argument("--results-dir", type=Path, default=PACKAGE / "results")
    parser.add_argument("--iou-floor", type=float, default=.90)
    parser.add_argument("--max-task-length", type=float, default=600.)
    parser.add_argument("--no-trim", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.iou_floor) or not 0 < args.iou_floor <= 1:
        parser.error("--iou-floor must be in (0, 1]")
    if not math.isfinite(args.max_task_length) or args.max_task_length <= 0:
        parser.error("--max-task-length must be positive and finite")
    if args.resume_session and (args.no_trim or args.iou_floor != .90 or args.max_task_length != 600.):
        parser.error("Resume uses recorded settings; start a new run to change them")
    hashes = source_hashes()
    if args.resume_session:
        session = args.resume_session.resolve()
        generation = verify_session(session, hashes)
    else:
        original = args.input_dir.resolve()
        for name in ("config.yaml", "target.stl"):
            if not (original / name).is_file():
                parser.error(f"Missing input: {original / name}")
        c.load_config(original / "config.yaml")
        session = args.results_dir.resolve() / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        (session / "input").mkdir(parents=True)
        for name in ("config.yaml", "target.stl"):
            shutil.copyfile(original / name, session / "input" / name)
        for name in hashes:
            destination = session / "source" / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PACKAGE / name, destination)
        settings = dict(iou_floor=args.iou_floor, screening_margin=.001,
                        max_task_length_mm=args.max_task_length, trim=not args.no_trim)
        r.write_json(session / "run.json", dict(source_sha256=hashes, environment=r.environment(),
            input_directory=str(original), settings=settings,
            input_sha256={name: r.sha256(session / "input" / name)
                          for name in ("config.yaml", "target.stl")}))
        print(f"Run: {session}", flush=True)
        try:
            generation = generate(session, settings)
        except Exception as exc:
            r.write_json(session / "summary.json", dict(status="ERROR", stage="generation",
                makespan_s=None, error=f"{type(exc).__name__}: {exc}"))
            print(f"Generation ERROR: {exc}", flush=True)
            return 1
    if args.prepare_only:
        print(f"Geometry only; allocation and full validation pending. Session: {session}", flush=True)
        return 0
    return allocate(session, generation, hashes)

"""Load immutable deposition tasks; never infer paths from a target STL."""

from __future__ import annotations

import gzip
import hashlib
import json
import math
import struct
from collections import Counter
from pathlib import Path

from .common import Task


def build_tasks(target, config):
    """Preserve the supplied polyline coordinates, direction and task boundaries."""
    folder = Path(target).parent
    source = folder / "deposition_paths.json"
    if not source.exists():
        source = folder / "deposition_paths.json.gz"
    if not source.is_file():
        raise FileNotFoundError(
            f"MISSING_PROVIDED_PATHS: {source}. This case cannot be scheduled from STL alone "
            "in assignment-only mode. Supply the original paths; no slicer fallback is allowed."
        )
    opener = gzip.open if source.suffix == ".gz" else open
    with opener(source, "rt", encoding="utf-8") as stream:
        payload = json.load(stream)
    if payload.get("schema_version") != 1:
        raise ValueError("Unsupported deposition path schema")
    actual_paths_sha = hashlib.sha256(json.dumps(
        payload["tasks"], sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()
    if actual_paths_sha != payload.get("tasks_sha256"):
        raise ValueError("Supplied path geometry checksum mismatch")
    provenance = payload["provenance"]
    for name in ("config.yaml", "target.stl"):
        actual = hashlib.sha256((folder / name).read_bytes()).hexdigest()
        if actual != provenance["input_sha256"][name]:
            raise ValueError(f"Supplied path/input mismatch: {name}")
    tasks = []
    for item in payload["tasks"]:
        layer = item["layer"]
        if type(layer) is not int or layer < 0:
            raise ValueError("Invalid supplied layer index")
        points = tuple(tuple(float(v) for v in p) for p in item["points_mm"])
        if len(points) < 2 or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError("Supplied paths require finite XYZ polylines")
        offset = 1.0 if config.process.tcp_z_reference == "top" else 0.5
        z = config.process.build_plane_z_mm + (layer + offset) * config.process.layer_height_mm
        if any(abs(p[2] - z) > 1e-6 for p in points):
            raise ValueError("Supplied path height differs from the configured layer")
        task = Task(item["task_id"], layer, points[0], points[-1], points[1:-1])
        if not isinstance(task.task_id, str) or not task.task_id or task.length <= 0:
            raise ValueError("Invalid supplied task identity or length")
        if any(math.dist(a, b) <= 1e-9 for a, b in zip(points, points[1:])):
            raise ValueError("Supplied paths contain zero-length edges")
        tasks.append(task)
    if not tasks or len({task.task_id for task in tasks}) != len(tasks):
        raise ValueError("Supplied tasks must be nonempty and uniquely identified")
    if len(tasks) != provenance["task_count"] or not math.isclose(
        sum(t.length for t in tasks), provenance["total_deposition_length_mm"],
        rel_tol=1e-10, abs_tol=1e-6,
    ):
        raise ValueError("Supplied task count/length differs from its provenance")
    return tasks


def verify_deposition(states, tasks):
    """Reject missing, duplicated, reversed or modified deposition edges."""
    def edge_key(a, b):
        # Compact keys avoid large nested tuple counters for dense supplied paths.
        return struct.pack("<6q", *(round(float(v) * 1e6) for v in (*a, *b)))

    expected = Counter(edge_key(a, b) for task in tasks
                       for a, b in zip(task.points, task.points[1:]))
    actual = Counter(edge_key(a.point, b.point)
                     for state in states.values()
                     for a, b in zip(state.records, state.records[1:]) if a.mode == "D")
    if actual != expected:
        raise ValueError("Scheduled deposition edges differ from the supplied path multiset")
    return dict(status="PASS", deposition_edges=sum(expected.values()),
                total_deposition_length_mm=sum(t.length for t in tasks),
                coordinate_tolerance_mm=1e-6)

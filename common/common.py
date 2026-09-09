"""Shared configuration, STL task generation and trajectory utilities."""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


PROJECT_DIR = Path(__file__).resolve().parents[2]
TESTS_DIR = PROJECT_DIR / "tests"

# Populated by load_config() before scheduling starts.
ROBOTS: dict[int, dict[str, object]] = {}
DEPOSITION_CLEARANCE_MM = 0.0
LAYER_HEIGHT_MM = 0.0
BUILD_PLANE_Z_MM = 0.0
CONFIG = None


Point = tuple[float, float, float]


@dataclass(frozen=True)
class Task:
    task_id: str
    layer: int
    start: Point
    end: Point
    via: tuple[Point, ...] = ()

    @property
    def points(self) -> tuple[Point, ...]:
        return (self.start, *self.via, self.end)

    @property
    def length(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:], strict=False))


@dataclass
class Record:
    robot_id: int
    time_s: float
    point: Point
    mode: str


@dataclass
class DepositReservation:
    robot_id: int
    start_s: float
    end_s: float
    start: Point
    end: Point


@dataclass
class RobotState:
    robot_id: int
    point: Point
    available_s: float = 0.0
    records: list[Record] = field(default_factory=list)


def _positive_float(value: object, field_name: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise ValueError(f"{field_name} must be greater than zero")
    return number


def _point(value: object, field_name: str) -> Point:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field_name} must contain exactly three coordinates")
    return float(value[0]), float(value[1]), float(value[2])


def load_config(path: Path) -> None:
    """Load robot geometry and process speeds from WAAM Validator config.yaml."""
    global ROBOTS, DEPOSITION_CLEARANCE_MM, LAYER_HEIGHT_MM, BUILD_PLANE_Z_MM
    global CONFIG
    from waam_validator.config.loader import load_config as load_validator_config
    CONFIG = load_validator_config(path)

    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("config.yaml root must be a mapping")

    process = data.get("process")
    robot_rows = data.get("robots")
    collision = data.get("collision", {})
    if not isinstance(process, dict):
        raise ValueError("config.yaml is missing the process section")
    if not isinstance(robot_rows, list) or len(robot_rows) != 3:
        raise ValueError("config.yaml must contain exactly three robots")
    if not isinstance(collision, dict):
        raise ValueError("config.yaml collision section must be a mapping")

    travel_speed = _positive_float(
        process.get("travel_speed_mm_s"), "process.travel_speed_mm_s"
    )
    deposition_speed = _positive_float(
        process.get("deposition_speed_mm_s"), "process.deposition_speed_mm_s"
    )
    LAYER_HEIGHT_MM = _positive_float(
        process.get("layer_height_mm"), "process.layer_height_mm"
    )
    BUILD_PLANE_Z_MM = float(process.get("build_plane_z_mm", 0.0))
    arm_clearance = max(0.0, float(collision.get("arm_clearance_mm", 0.0)))

    loaded: dict[int, dict[str, object]] = {}
    maximum_tcp_radius = 0.0
    for row in robot_rows:
        if not isinstance(row, dict):
            raise ValueError("Each robots entry must be a mapping")
        robot_id = int(row["id"])
        if robot_id in loaded:
            raise ValueError(f"Duplicate robot id: {robot_id}")
        tcp_radius = max(0.0, float(row.get("tcp_radius_mm", 0.0)))
        maximum_tcp_radius = max(maximum_tcp_radius, tcp_radius)
        loaded[robot_id] = {
            "base": _point(row.get("base_xyz_mm"), f"robot {robot_id}.base_xyz_mm"),
            "home": _point(row.get("home_xyz_mm"), f"robot {robot_id}.home_xyz_mm"),
            "reach_mm": _positive_float(
                row.get("xy_reach_radius_mm"), f"robot {robot_id}.xy_reach_radius_mm"
            ),
            "tcp_radius_mm": tcp_radius,
            "travel_speed_mm_s": travel_speed,
            "deposition_speed_mm_s": deposition_speed,
        }

    if set(loaded) != {1, 2, 3}:
        raise ValueError("Robot IDs in config.yaml must be 1, 2, and 3")

    ROBOTS = loaded
    # Conservative simplified TCP clearance used by this heuristic. The
    # Validator will still perform its more detailed capsule collision check.
    DEPOSITION_CLEARANCE_MM = 2.0 * maximum_tcp_radius + arm_clearance


def distance_3d(a: Point, b: Point) -> float:
    return math.dist(a, b)


def distance_xy(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance_xy(p: Point, a: Point, b: Point) -> float:
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    px, py = p[0], p[1]
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect_xy(a: Point, b: Point, c: Point, d: Point) -> bool:
    eps = 1e-9
    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
       ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return (
        (abs(o1) <= eps and point_segment_distance_xy(c, a, b) <= eps)
        or (abs(o2) <= eps and point_segment_distance_xy(d, a, b) <= eps)
        or (abs(o3) <= eps and point_segment_distance_xy(a, c, d) <= eps)
        or (abs(o4) <= eps and point_segment_distance_xy(b, c, d) <= eps)
    )


def segment_distance_xy(a: Point, b: Point, c: Point, d: Point) -> float:
    if segments_intersect_xy(a, b, c, d):
        return 0.0
    return min(
        point_segment_distance_xy(a, c, d),
        point_segment_distance_xy(b, c, d),
        point_segment_distance_xy(c, a, b),
        point_segment_distance_xy(d, a, b),
    )


def is_reachable(robot_id: int, task: Task) -> bool:
    base = ROBOTS[robot_id]["base"]
    reach = ROBOTS[robot_id]["reach_mm"]
    # WAAM Validator evaluates reach in the XY plane.
    return all(distance_xy(base, point) <= reach for point in task.points)


def read_tasks(path: Path) -> list[Task]:
    required = {
        "task_id", "layer", "start_x_mm", "start_y_mm", "start_z_mm",
        "end_x_mm", "end_y_mm", "end_z_mm",
    }
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"tasks.csv missing columns: {sorted(missing)}")
        tasks = [
            Task(
                task_id=row["task_id"].strip(),
                layer=int(row["layer"]),
                start=(float(row["start_x_mm"]), float(row["start_y_mm"]), float(row["start_z_mm"])),
                end=(float(row["end_x_mm"]), float(row["end_y_mm"]), float(row["end_z_mm"])),
            )
            for row in reader
        ]
    if not tasks:
        raise ValueError("tasks.csv contains no tasks")
    return tasks


def build_benchmark_tasks(case_number: int) -> list[Task]:
    """Slice the actual benchmark STL, including every target layer."""
    from .slicer import build_tasks
    return build_tasks(TESTS_DIR / f"{case_number:02d}" / "target.stl", CONFIG)


def build_legacy_benchmark_tasks(case_number: int) -> list[Task]:
    """Historical synthetic examples; not the current tests/01-06 target geometry."""
    try:
        from scripts.generate_benchmark_jobs import BENCHMARKS
    except ImportError as exc:
        raise RuntimeError(
            "Could not import scripts/generate_benchmark_jobs.py. "
            "Run this script from inside the WAAM_Validator project."
        ) from exc

    spec = next((item for item in BENCHMARKS if item.number == case_number), None)
    if spec is None:
        raise ValueError(f"Benchmark case {case_number} is not defined")

    tasks: list[Task] = []
    for layer in range(spec.layers):
        z_mm = BUILD_PLANE_Z_MM + (layer + 0.5) * LAYER_HEIGHT_MM
        for path_index, path in enumerate(spec.factory(), start=1):
            coordinates = list(path.coords)
            for segment_index, (left, right) in enumerate(
                zip(coordinates[:-1], coordinates[1:], strict=True), start=1
            ):
                tasks.append(
                    Task(
                        task_id=f"L{layer:03d}-P{path_index:03d}-S{segment_index:03d}",
                        layer=layer,
                        start=(float(left[0]), float(left[1]), z_mm),
                        end=(float(right[0]), float(right[1]), z_mm),
                    )
                )
    return tasks


def nonconflicting_start(
    robot_id: int,
    task: Task,
    earliest_s: float,
    duration_s: float,
    reservations: list[DepositReservation],
) -> float:
    """Delay a deposition until it no longer overlaps a nearby deposition."""
    start_s = earliest_s
    while True:
        end_s = start_s + duration_s
        blockers = [
            r for r in reservations
            if r.robot_id != robot_id
            and start_s < r.end_s
            and r.start_s < end_s
            and segment_distance_xy(task.start, task.end, r.start, r.end)
                < DEPOSITION_CLEARANCE_MM
        ]
        if not blockers:
            return start_s
        start_s = max(r.end_s for r in blockers)


def candidate_plan(
    robot: RobotState,
    task: Task,
    layer_release_s: float,
    reservations: list[DepositReservation],
) -> tuple[float, float, float, float]:
    cfg = ROBOTS[robot.robot_id]
    ready_s = max(robot.available_s, layer_release_s)
    travel_s = distance_3d(robot.point, task.start) / cfg["travel_speed_mm_s"]
    arrival_s = ready_s + travel_s
    deposit_duration_s = distance_3d(task.start, task.end) / cfg["deposition_speed_mm_s"]
    deposit_start_s = nonconflicting_start(
        robot.robot_id, task, arrival_s, deposit_duration_s, reservations
    )
    finish_s = deposit_start_s + deposit_duration_s
    return finish_s, ready_s, arrival_s, deposit_start_s


def add_record(records: list[Record], record: Record) -> None:
    """Avoid duplicate timestamps by changing the mode at an existing waypoint."""
    if records and math.isclose(records[-1].time_s, record.time_s, rel_tol=0.0, abs_tol=1e-9):
        if distance_3d(records[-1].point, record.point) > 1e-9:
            raise ValueError("Two different positions were assigned at the same time")
        records[-1] = record
    else:
        records.append(record)


def commit_task(
    robot: RobotState,
    task: Task,
    ready_s: float,
    arrival_s: float,
    deposit_start_s: float,
    finish_s: float,
    reservations: list[DepositReservation],
) -> None:
    # Remain stationary until the robot is allowed to start moving.
    if ready_s > robot.available_s:
        add_record(robot.records, Record(robot.robot_id, ready_s, robot.point, "T"))
    else:
        add_record(robot.records, Record(robot.robot_id, robot.available_s, robot.point, "T"))

    if arrival_s > ready_s:
        add_record(robot.records, Record(robot.robot_id, arrival_s, task.start, "W"))
    else:
        add_record(robot.records, Record(robot.robot_id, ready_s, task.start, "W"))

    add_record(robot.records, Record(robot.robot_id, deposit_start_s, task.start, "D"))
    add_record(robot.records, Record(robot.robot_id, finish_s, task.end, "W"))

    reservations.append(
        DepositReservation(robot.robot_id, deposit_start_s, finish_s, task.start, task.end)
    )
    robot.point = task.end
    robot.available_s = finish_s


def write_trajectory(states: dict[int, RobotState], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["robot_id", "time_s", "x_mm", "y_mm", "z_mm", "mode"])
        for rid in sorted(states):
            for r in states[rid].records:
                writer.writerow(
                    [r.robot_id, f"{r.time_s:.6f}", f"{r.point[0]:.6f}",
                     f"{r.point[1]:.6f}", f"{r.point[2]:.6f}", r.mode]
                )


def write_assignments(assignments: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["task_id", "layer", "robot_id", "deposit_start_s", "finish_s"],
        )
        writer.writeheader()
        for row in assignments:
            writer.writerow(row)

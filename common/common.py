"""Shared configuration, STL task generation and trajectory utilities."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# Populated by load_config() before scheduling starts.
ROBOTS: dict[int, dict[str, object]] = {}
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
class RobotState:
    robot_id: int
    point: Point
    available_s: float = 0.0
    records: list[Record] = field(default_factory=list)


def _positive_float(value: object, field_name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{field_name} must be greater than zero")
    return number


def _point(value: object, field_name: str) -> Point:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field_name} must contain exactly three coordinates")
    return float(value[0]), float(value[1]), float(value[2])


def load_config(path: Path) -> None:
    """Load robot geometry and process speeds from WAAM Validator config.yaml."""
    global ROBOTS
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

    travel_speed = _positive_float(process.get("travel_speed_mm_s"), "process.travel_speed_mm_s")
    deposition_speed = _positive_float(
        process.get("deposition_speed_mm_s"), "process.deposition_speed_mm_s"
    )

    loaded: dict[int, dict[str, object]] = {}
    for row in robot_rows:
        if not isinstance(row, dict):
            raise ValueError("Each robots entry must be a mapping")
        robot_id = int(row["id"])
        if robot_id in loaded:
            raise ValueError(f"Duplicate robot id: {robot_id}")
        tcp_radius = max(0.0, float(row.get("tcp_radius_mm", 0.0)))
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


def distance_3d(a: Point, b: Point) -> float:
    return math.dist(a, b)


def distance_xy(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def add_record(records: list[Record], record: Record) -> None:
    """Avoid duplicate timestamps by changing the mode at an existing waypoint."""
    if records and math.isclose(records[-1].time_s, record.time_s, rel_tol=0.0, abs_tol=1e-9):
        if distance_3d(records[-1].point, record.point) > 1e-9:
            raise ValueError("Two different positions were assigned at the same time")
        records[-1] = record
    else:
        records.append(record)


def write_trajectory(states: dict[int, RobotState], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["robot_id", "time_s", "x_mm", "y_mm", "z_mm", "mode"])
        for rid in sorted(states):
            for r in states[rid].records:
                writer.writerow(
                    [
                        r.robot_id,
                        f"{r.time_s:.6f}",
                        f"{r.point[0]:.6f}",
                        f"{r.point[1]:.6f}",
                        f"{r.point[2]:.6f}",
                        r.mode,
                    ]
                )


def write_assignments(assignments: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "task_id",
                "layer",
                "robot_id",
                "deposit_start_s",
                "finish_s",
                "job_start_s",
                "job_finish_s",
                "approach_time_s",
                "return_home_time_s",
                "transition_from_task",
            ],
        )
        writer.writeheader()
        for row in assignments:
            writer.writerow(row)

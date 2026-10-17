"""Continuous XY clearance via short swept motion segments and time reservations.

Each segment reserves its full swept region only for its own time interval.
This is conservative between waypoints (not point sampling), but does not
reserve an entire task footprint for the entire job duration.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np
from shapely import STRtree, bounds, box, convex_hull, distance, linestrings, multipoints

TIME_PAD_S = 1e-5  # Also covers the six-decimal trajectory timestamp export.


@dataclass(frozen=True)
class MotionSegment:
    start_s: float
    finish_s: float
    start: tuple[float, float, float]
    end: tuple[float, float, float]


@dataclass
class SweptGeometry:
    token: int
    arm: object
    tcp: object
    arm_tree: object
    tcp_tree: object
    arm_bounds: object
    tcp_bounds: object


def build_motion(approach, deposition, retreat, process, max_xy_step_mm):
    """Use the same piecewise linear motion, speeds and setup times as render()."""
    if not math.isfinite(max_xy_step_mm) or max_xy_step_mm <= 0:
        raise ValueError("max_tcp_step_mm must be finite and positive")
    segments = []
    time = 0.0

    def move(points, speed):
        nonlocal time
        for left, right in zip(points, points[1:]):
            length = math.dist(left, right)
            if length < 1e-9:
                continue
            duration = length / speed
            pieces = max(1, math.ceil(math.dist(left[:2], right[:2]) / max_xy_step_mm))
            for index in range(pieces):
                a, b = index / pieces, (index + 1) / pieces
                start = tuple(x + a * (y - x) for x, y in zip(left, right, strict=True))
                end = tuple(x + b * (y - x) for x, y in zip(left, right, strict=True))
                segments.append(MotionSegment(time + a * duration, time + b * duration, start, end))
            time += duration

    def wait(point, duration):
        nonlocal time
        if duration > 0:
            segments.append(MotionSegment(time, time + duration, point, point))
            time += duration

    move(approach, process.travel_speed_mm_s)
    wait(deposition[0], process.arc_on_time_s or 0.0)
    move(deposition, process.deposition_speed_mm_s)
    wait(deposition[-1], process.arc_off_time_s or 0.0)
    move(retreat, process.travel_speed_mm_s)
    return tuple(segments)


def merge_intervals(intervals):
    merged = []
    for low, high in sorted(intervals):
        if merged and low <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], high))
        else:
            merged.append((low, high))
    return tuple(merged)


class TemporalConflicts:
    """Lazy, bounded cache of forbidden relative start offsets for each route pair."""

    def __init__(self, routes, config, broad_conflict, cache_size=8192):
        self.routes = routes
        self.config = config
        self.robots = {robot.id: robot for robot in config.robots}
        self.broad_conflict = broad_conflict
        self.cache_size = cache_size
        self.cache = OrderedDict()
        self.geometry = OrderedDict()
        self.geometry_segments = 0
        self.geometry_limit = 120_000
        self.geometry_token = 0
        self.spatial_pairs = OrderedDict()
        self.pair_count = 0
        self.pair_limit = 1_000_000

    def _geometry(self, key, plan, rid):
        # XY geometry is independent of layer height, elapsed times and wait
        # duration. Reuse exact matching coordinates without rounding them.
        spatial = getattr(plan, "spatial_key", ())
        key = (rid, spatial or key)
        if key in self.geometry:
            self.geometry.move_to_end(key)
            return self.geometry[key]
        base = self.robots[rid].base_xyz_mm[:2]
        starts = np.array([s.start[:2] for s in plan.motion], dtype=float).reshape(-1, 2)
        ends = np.array([s.end[:2] for s in plan.motion], dtype=float).reshape(-1, 2)
        triangles = np.stack((np.broadcast_to(base, starts.shape), starts, ends), axis=1)
        arm = convex_hull(multipoints(triangles))
        tcp = np.empty(len(starts), dtype=object)
        moving = np.any(starts != ends, axis=1)
        tcp[moving] = linestrings(np.stack((starts[moving], ends[moving]), axis=1))
        tcp[~moving] = multipoints(starts[~moving, None, :])
        self.geometry_token += 1
        value = SweptGeometry(
            self.geometry_token, arm, tcp, STRtree(arm), STRtree(tcp), bounds(arm), bounds(tcp)
        )
        self.geometry[key] = value
        self.geometry_segments += len(arm)
        while (len(self.geometry) > 2048 or self.geometry_segments > self.geometry_limit) and len(
            self.geometry
        ) > 1:
            _, expired = self.geometry.popitem(last=False)
            self.geometry_segments -= len(expired.arm)
        return value

    @staticmethod
    def _near_pairs(left, right, tree, clearance, left_bounds):
        """Broad phase via tree boxes, then exact continuous swept-region distance."""
        boxes = box(
            left_bounds[:, 0] - clearance,
            left_bounds[:, 1] - clearance,
            left_bounds[:, 2] + clearance,
            left_bounds[:, 3] + clearance,
        )
        pairs = tree.query(boxes)
        if pairs.shape[1] == 0:
            return np.empty(0, dtype=np.int64)
        a, b = pairs
        keep = distance(left[a], right[b]) <= clearance
        return a[keep] * len(right) + b[keep]

    def _colliding_pairs(self, left, right, rid, sid):
        key = (left.token, right.token)
        if key in self.spatial_pairs:
            self.spatial_pairs.move_to_end(key)
            return self.spatial_pairs[key]
        a, b = self.robots[rid], self.robots[sid]
        guard = max(0.01, self.config.collision.geometry_epsilon_mm)
        pairs = []
        if self.config.collision.check_arm_envelope:
            clearance = (
                a.arm_envelope_radius_mm
                + b.arm_envelope_radius_mm
                + self.config.collision.arm_clearance_mm
                + guard
            )
            pairs.append(
                self._near_pairs(left.arm, right.arm, right.arm_tree, clearance, left.arm_bounds)
            )
        if self.config.collision.check_tcp_radius:
            clearance = a.tcp_radius_mm + b.tcp_radius_mm + guard
            pairs.append(
                self._near_pairs(left.tcp, right.tcp, right.tcp_tree, clearance, left.tcp_bounds)
            )
        codes = np.unique(np.concatenate(pairs)) if pairs else np.empty(0, dtype=np.int64)
        result = (
            (codes // len(right.arm), codes % len(right.arm)) if len(right.arm) else (codes, codes)
        )
        if len(codes) <= self.pair_limit:
            self.spatial_pairs[key] = result
            self.pair_count += len(codes)
            while len(self.spatial_pairs) > 2048 or self.pair_count > self.pair_limit:
                _, expired = self.spatial_pairs.popitem(last=False)
                self.pair_count -= len(expired[0])
        return result

    @staticmethod
    def _time_windows(left, right, pairs):
        a, b = pairs
        if len(a) == 0:
            return ()
        ls = np.fromiter((s.start_s for s in left.motion), dtype=float)
        lf = np.fromiter((s.finish_s for s in left.motion), dtype=float)
        rs = np.fromiter((s.start_s for s in right.motion), dtype=float)
        rf = np.fromiter((s.finish_s for s in right.motion), dtype=float)
        low = rs[b] - lf[a] - TIME_PAD_S
        high = rf[b] - ls[a] + TIME_PAD_S
        order = np.lexsort((high, low))
        low, high = low[order], np.maximum.accumulate(high[order])
        first = np.r_[0, np.flatnonzero(low[1:] > high[:-1]) + 1]
        last = np.r_[first[1:] - 1, len(low) - 1]
        return tuple(zip(low[first].tolist(), high[last].tolist(), strict=True))

    def forbidden_offsets(self, i, rid, j, sid):
        """Forbidden values of (start of route i - start of route j)."""
        return self.offsets((i, rid), self.routes[i][rid], rid, (j, sid), self.routes[j][sid], sid)

    def offsets(self, left_key, left, rid, right_key, right, sid):
        """Also support origin-dependent routes without a fixed route table."""
        if left_key > right_key:
            return tuple(
                (-high, -low)
                for low, high in reversed(self.offsets(right_key, right, sid, left_key, left, rid))
            )
        key = (left_key, right_key)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        if not self.broad_conflict(left, right):
            intervals = ()
        else:
            left_geometry = self._geometry(left_key, left, rid)
            right_geometry = self._geometry(right_key, right, sid)
            intervals = self._time_windows(
                left, right, self._colliding_pairs(left_geometry, right_geometry, rid, sid)
            )
        self.cache[key] = intervals
        if len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return intervals

    def earliest_start(self, i, rid, earliest_s, duration_s, jobs, *, include_same_robot=False):
        forbidden = []
        for job in jobs:
            if job.finish_s < earliest_s - TIME_PAD_S:
                continue
            if job.robot_id == rid:
                if include_same_robot:
                    # Same robot jobs may touch at their common home endpoint.
                    forbidden.append((job.start_s - duration_s, job.finish_s))
                continue
            forbidden.extend(
                (job.start_s + low, job.start_s + high)
                for low, high in self.forbidden_offsets(i, rid, job.task_index, job.robot_id)
            )
        start = earliest_s
        for low, high in sorted(forbidden):
            # Open interval boundaries already contain TIME_PAD_S for other robots.
            if low >= start:
                break
            if start < high:
                start = high
        return start

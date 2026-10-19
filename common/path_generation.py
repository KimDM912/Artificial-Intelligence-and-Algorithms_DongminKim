"""Generate contour/hatch candidates from an arbitrary target STL slice."""
from __future__ import annotations

import math

from shapely import LineString, get_parts, union_all
from shapely.affinity import rotate
from shapely.ops import substring

from waam_validator.shape.metrics import compute_shape_metrics
from waam_validator.shape.polygon_utils import normalize_polygon

from . import common as c
from .feasibility import safe_robots
from .trajectory_precision import clean_deposition_points


def clean_points(points):
    result = []
    for point in points:
        point = tuple(float(v) for v in point)
        if not result or math.dist(result[-1], point) > 1e-8:
            result.append(point)
    return result


def split_length(points, maximum):
    """Partition long polylines without dropping edges or adding deposition."""
    current, length = [points[0]], 0.0
    for end in points[1:]:
        start = current[-1]
        distance = math.dist(start, end)
        while distance > 1e-8 and length + distance > maximum + 1e-8:
            fraction = (maximum - length) / distance
            middle = tuple(a + fraction * (b - a) for a, b in zip(start, end))
            current.append(middle)
            yield current
            current, length, start = [middle], 0.0, middle
            distance = math.dist(start, end)
        if distance > 1e-8:
            current.append(end)
            length += distance
        if length >= maximum - 1e-8:
            yield current
            current, length = [end], 0.0
    if len(current) > 1:
        yield current


def split_reachable(points, config):
    """Convex XY reach disks also contain every segment between its vertices."""
    if safe_robots(points, config):
        yield points
        return
    if len(points) == 2:
        if math.dist(*points) < 0.5:
            raise ValueError("Generated deposition includes an unreachable region")
        middle = tuple((a + b) / 2 for a, b in zip(*points))
        yield from split_reachable([points[0], middle], config)
        yield from split_reachable([middle, points[1]], config)
    else:
        middle = len(points) // 2
        yield from split_reachable(points[:middle + 1], config)
        yield from split_reachable(points[middle:], config)


def make_tasks(paths, layer, config, max_length):
    z = config.process.build_plane_z_mm + (
        layer + (1.0 if config.process.tcp_z_reference == "top" else .5)
    ) * config.process.layer_height_mm
    tasks = []
    for path in paths:
        points = clean_points([(p[0], p[1], z) for p in path])
        if len(points) < 2:
            continue
        for chunk in split_length(points, max_length):
            for reachable in split_reachable(chunk, config):
                reachable = clean_deposition_points(reachable, config)
                if len(reachable) < 2:
                    continue
                tasks.append(c.Task(f"L{layer:03d}-G{len(tasks):05d}", layer,
                                    reachable[0], reachable[-1], tuple(reachable[1:-1])))
    return tasks


def rings(geometry, tolerance):
    for polygon in sorted(get_parts(geometry), key=lambda p: p.bounds):
        if polygon.geom_type != "Polygon":
            continue
        for ring in (polygon.exterior, *polygon.interiors):
            coords = clean_points(ring.simplify(tolerance).coords)
            if len(coords) > 1:
                yield coords


def contour_paths(target, width, spacing_ratio, resolution, inset_ratio=.475):
    """Parallel contours with substantially less overlap than half-bead spacing."""
    inset = width * inset_ratio
    paths = []
    while True:
        interior = target.buffer(-inset, quad_segs=resolution)
        if interior.is_empty:
            return paths
        paths.extend(rings(interior, width * .00625))
        inset += width * spacing_ratio


def deduplicated_contours(target, width, resolution):
    """Remove near-coincident opposing offset passes in narrow residual strips.

    Offsetting a thin residual region produces a closed contour that can print
    almost the same centerline twice. Clip only near already retained lines;
    the complete resulting layer still has to pass the geometry screen.
    """
    paths = contour_paths(target, width, .98, resolution, inset_ratio=.49)
    covered, result = None, []
    for path in paths:
        line = LineString(path)
        count = max(1, math.ceil(line.length / 100.0))
        for index in range(count):
            part = substring(line, index * line.length / count, (index + 1) * line.length / count)
            remaining = part if covered is None else part.difference(covered)
            additions = []
            for piece in get_parts(remaining):
                if piece.geom_type == "LineString" and piece.length > 1e-6:
                    coords = list(piece.coords)
                    # Restore only short gaps introduced by the clipping chunks.
                    # These are explicit D connectors, included in shape and time.
                    gap = math.dist(result[-1][-1], coords[0]) if result else math.inf
                    if gap < 1e-7:
                        result[-1].extend(coords[1:])
                    elif gap <= width * .30 and target.covers(LineString([result[-1][-1], coords[0]])):
                        bridge = LineString([result[-1][-1], coords[0]])
                        additions.append(bridge.buffer(width * .15, quad_segs=resolution))
                        result[-1].extend(coords)
                    else:
                        result.append(coords)
                    additions.append(piece.buffer(width * .15, quad_segs=resolution))
            if additions:
                new = union_all(additions)
                covered = new if covered is None else covered.union(new)
    return connect_nearby(result, target, width)


def connect_nearby(paths, target, width):
    """Join nearby path ends through material, counting each bridge as deposition.

    This reduces arc cycles introduced by contour clipping. Paths may reverse
    because this experiment generates new geometry rather than preserving an
    existing directed input. No connector may cross a target hole or gap.
    """
    remaining = [list(path) for path in paths]
    interior = target.buffer(-width * .45)
    result = []
    while remaining:
        chain = remaining.pop(0)
        while remaining:
            candidates = []
            for i, other in enumerate(remaining):
                for reverse_chain in (False, True):
                    end = chain[0] if reverse_chain else chain[-1]
                    for reverse_other in (False, True):
                        start = other[-1] if reverse_other else other[0]
                        gap = math.dist(end, start)
                        if gap <= 1.5 * width:
                            candidates.append((gap, i, reverse_chain, reverse_other, end, start))
            match = None
            for candidate in sorted(candidates):
                if candidate[0] < 1e-8 or interior.covers(LineString(candidate[-2:])):
                    match = candidate
                    break
            if match is None:
                break
            _, i, reverse_chain, reverse_other, _, _ = match
            other = remaining.pop(i)
            if reverse_chain:
                chain.reverse()
            if reverse_other:
                other.reverse()
            chain = clean_points([*chain, *other])
        result.append(chain)
    return result


def hatch_paths(target, width, angle, resolution):
    """One boundary pass plus clipped interior hatch, with short safe connectors.

    Connections remain inside the eroded material region; they never bridge a
    hole or a disconnected island. Long paths are split later for robot access.
    """
    center = target.buffer(-width * .475, quad_segs=resolution)
    paths = list(rings(center, width * .00625))
    core = target.buffer(-width * .95, quad_segs=resolution)
    if core.is_empty:
        return paths
    rotated = rotate(core, -angle, origin=(0, 0))
    xmin, ymin, xmax, ymax = rotated.bounds
    count = max(1, math.ceil((ymax - ymin) / (width * .95)))
    rows = []
    for row in range(count):
        y = ymin + (row + .5) * (ymax - ymin) / count
        clipped = rotated.intersection(LineString([(xmin - width, y), (xmax + width, y)]))
        segments = sorted((g for g in get_parts(clipped) if g.geom_type == "LineString"
                           and g.length > 1e-6), key=lambda g: g.bounds)
        if row % 2:
            segments.reverse()
        for segment in segments:
            coords = sorted(segment.coords, key=lambda p: p[0], reverse=bool(row % 2))
            world = list(rotate(LineString(coords), angle, origin=(0, 0)).coords)
            rows.append(world)
    chain = []
    for segment in rows:
        if chain:
            connector = LineString([chain[-1], segment[0]])
            if connector.length <= 1.5 * width and center.covers(connector):
                chain.extend(segment)
                continue
            paths.append(clean_points(chain))
        chain = list(segment)
    if chain:
        paths.append(clean_points(chain))
    return paths


def footprint(tasks, config):
    """Match the Validator's per-D-interval round buffers and snap precision."""
    shape = config.shape_validation
    segments = [LineString([a[:2], b[:2]]).buffer(
        config.process.bead_width_mm / 2, quad_segs=shape.polygon_buffer_resolution,
        cap_style="round", join_style="round")
        for task in tasks for a, b in zip(task.points, task.points[1:])]
    return normalize_polygon(union_all(segments), shape.polygon_snap_tolerance_mm,
                             shape.area_epsilon_mm2)


def select_layer(target, layer, config, *, iou_floor=.97, max_length=600.0):
    """Choose by deposition plus arc time, NOT by claimed final makespan.

    Identical geometry, speed and allocation rules are used after selection.
    Travel, collisions and workload balance require the subsequent scheduler.
    """
    width = config.process.bead_width_mm
    resolution = config.shape_validation.polygon_buffer_resolution
    generators = [(f"contour_{ratio:.2f}", lambda ratio=ratio: contour_paths(
        target, width, ratio, resolution)) for ratio in (.95, .75, .50)]
    generators += [(f"hatch_{angle:03d}", lambda angle=angle: hatch_paths(
        target, width, angle, resolution)) for angle in (0, 45, 90, 135)]
    generators.append(("contour_deduplicated", lambda: deduplicated_contours(target, width, resolution)))
    candidates, best = [], None
    for name, generate in generators:
        try:
            tasks = make_tasks(generate(), layer, config, max_length)
            if not tasks:
                raise ValueError("No printable paths")
            deposited = footprint(tasks, config)
            metrics, _ = compute_shape_metrics({layer: deposited}, {layer: target}, config)
            length = sum(t.length for t in tasks)
            cost = length / config.process.deposition_speed_mm_s + len(tasks) * (
                (config.process.arc_on_time_s or 0) + (config.process.arc_off_time_s or 0))
            accepted = metrics.passed and metrics.iou >= iou_floor
            record = dict(name=name, accepted=accepted, tasks=len(tasks), length_mm=length,
                          processing_robot_s=cost, iou=metrics.iou, coverage=metrics.coverage,
                          overfill_ratio=metrics.overfill_ratio)
            if accepted and (best is None or (cost, name) < best[0]):
                best = ((cost, name), tasks, deposited, record)
        except ValueError as exc:
            record = dict(name=name, accepted=False, error=str(exc))
        candidates.append(record)
    if best is None:
        raise ValueError(f"Layer {layer}: no candidate meets shape/reach criteria: {candidates}")
    return best[1], best[2], dict(layer=layer, selected=best[3], candidates=candidates)


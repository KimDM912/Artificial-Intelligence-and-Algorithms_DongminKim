"""Geometry-screened endpoint trimming independent of benchmark identity."""
import math

from shapely import LineString, union_all

from . import path_generation as p
from .trajectory_precision import clean_deposition_points


def trim(task, fraction, config):
    """Retain the middle arc-length interval without bridging discarded paths."""
    if not 0 <= fraction < 1:
        raise ValueError("Invalid trim fraction")
    lower, upper = task.length * fraction / 2, task.length * (1 - fraction / 2)
    points, distance = [], 0.
    for a, b in zip(task.points, task.points[1:]):
        length = math.dist(a, b)
        left, right = max(lower, distance), min(upper, distance + length)
        if right - left > 1e-9:
            for offset in (left, right):
                ratio = max(0., min(1., (offset - distance) / length))
                point = tuple(x + (y - x) * ratio for x, y in zip(a, b))
                if not points or math.dist(points[-1], point) > 1e-9:
                    points.append(point)
        distance += length
    points = clean_deposition_points(points, config)
    if len(points) < 2:
        raise ValueError("Trimming removed a complete task")
    return p.c.Task(task.task_id, task.layer, points[0], points[-1], tuple(points[1:-1]))


def proxy(tasks, config):
    shape = config.shape_validation
    geometry = union_all([LineString([v[:2] for v in t.points]).buffer(
        config.process.bead_width_mm / 2,
        quad_segs=shape.polygon_buffer_resolution) for t in tasks])
    return p.normalize_polygon(geometry, shape.polygon_snap_tolerance_mm, shape.area_epsilon_mm2)


def select_layer(tasks, target, config, floor, iterations=10, margin=.001):
    """Search with a fixed-denominator proxy, then verify exact segment buffers.

    The proxy is not certification. Cleanup and polygon discretization can
    affect monotonicity, so failed exact checks back off toward the full path.
    Every task and layer remains present; no omission allowance is consumed.
    """
    required = min(1., floor + margin)
    original = [trim(t, 0., config) for t in tasks]
    denominator = target.union(proxy(original, config)).area
    if denominator <= 0:
        raise ValueError("Empty target layer")

    def candidate(fraction):
        return [trim(t, fraction, config) for t in tasks]

    low, high = 0., .5
    for _ in range(iterations):
        middle = (low + high) / 2
        selected = candidate(middle)
        bound = target.intersection(proxy(selected, config)).area / denominator
        if bound >= required:
            low = middle
        else:
            high = middle
    for _ in range(15):
        selected = candidate(low)
        geometry = p.footprint(selected, config)
        shape, _ = p.compute_shape_metrics({tasks[0].layer: geometry},
                                          {tasks[0].layer: target}, config)
        if shape.passed and shape.iou >= required:
            if not all(p.safe_robots(t.points, config) for t in selected):
                raise ValueError("Trimmed path is unreachable")
            return selected, geometry, dict(
                layer=tasks[0].layer, trim_fraction=low, iou=shape.iou,
                coverage=shape.coverage, overfill_ratio=shape.overfill_ratio,
                original_length_mm=sum(t.length for t in tasks),
                retained_length_mm=sum(t.length for t in selected))
        low = low / 2 if low > 1e-6 else 0.
    raise ValueError(f"Layer {tasks[0].layer}: exact geometry screen failed")


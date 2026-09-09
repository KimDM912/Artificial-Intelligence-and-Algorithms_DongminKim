"""Deterministic contour-parallel bead paths from actual STL sections."""

from __future__ import annotations

from shapely import LineString, MultiPoint, get_parts
from shapely import Point as XYPoint

from waam_validator.shape.target import (
    determine_evaluation_layers,
    load_target_mesh,
    slice_target_layers,
)

from . import common as c


def safe_robots(points, config):
    """Conservative continuous XY sweep against the other robots parked at home.

    A moving Base-to-TCP capsule lies inside the convex hull of its base and
    TCP path. Vertical travel does not change this XY envelope.
    """
    feasible = []
    for robot in config.robots:
        base = robot.base_xyz_mm
        home = robot.home_xyz_mm
        path = [home, *points]
        if any(c.distance_xy(base, p) > robot.xy_reach_radius_mm - 0.01 for p in path):
            continue
        arm_sweep = MultiPoint([base[:2], *(p[:2] for p in path)]).convex_hull
        tcp_sweep = MultiPoint([p[:2] for p in path]).convex_hull
        safe = True
        for other in config.robots:
            if other.id == robot.id:
                continue
            parked = LineString([other.base_xyz_mm[:2], other.home_xyz_mm[:2]])
            required = (
                robot.arm_envelope_radius_mm
                + other.arm_envelope_radius_mm
                + config.collision.arm_clearance_mm
                + 0.01
            )
            if (
                config.collision.check_arm_envelope
                and arm_sweep.distance(parked) <= required
                or config.collision.check_tcp_radius
                and tcp_sweep.distance(XYPoint(other.home_xyz_mm[:2]))
                <= robot.tcp_radius_mm + other.tcp_radius_mm + 0.01
            ):
                safe = False
                break
        if safe:
            feasible.append(robot.id)
    return feasible


def split_safe_paths(points, config):
    """Bisect only infeasible contours; retain every original deposition edge."""
    if len(points) < 2:
        return
    if safe_robots(points, config):
        yield points
        return
    if len(points) == 2:
        # Split a long straight edge across overlapping robot workspaces.
        if c.distance_3d(points[0], points[1]) <= 1.0:
            raise ValueError(f"No safe reachable robot for edge {points}")
        midpoint = tuple((a + b) / 2 for a, b in zip(*points, strict=True))
        yield from split_safe_paths([points[0], midpoint], config)
        yield from split_safe_paths([midpoint, points[1]], config)
        return
    middle = len(points) // 2
    yield from split_safe_paths(points[: middle + 1], config)
    yield from split_safe_paths(points[middle:], config)


def build_tasks(target, config):
    mesh = load_target_mesh(target, config)
    layers = slice_target_layers(mesh, determine_evaluation_layers(mesh, {}, config), config)
    width = config.process.bead_width_mm
    tasks = []
    for layer, section in layers.items():
        z = (
            config.process.build_plane_z_mm
            + (layer + (1.0 if config.process.tcp_z_reference == "top" else 0.5))
            * config.process.layer_height_mm
        )
        # Slight overlap preserves thin nominal bead-width walls after STL rounding.
        inset = width * 0.475
        contour_index = 0
        while True:
            interior = section.buffer(-inset, quad_segs=8)
            if interior.is_empty:
                break
            for polygon in sorted(get_parts(interior), key=lambda p: p.bounds):
                if polygon.geom_type != "Polygon":
                    continue
                for ring in [polygon.exterior, *polygon.interiors]:
                    coords = list(ring.simplify(width * 0.0125).coords)
                    points = [(float(coords[0][0]), float(coords[0][1]), z)]
                    for x, y in coords[1:]:
                        point = (float(x), float(y), z)
                        if c.distance_3d(points[-1], point) >= 1e-4:
                            points.append(point)
                    for group in split_safe_paths(points, config):
                        tasks.append(
                            c.Task(
                                f"L{layer:03d}-C{contour_index:05d}",
                                layer,
                                group[0],
                                group[-1],
                                tuple(group[1:-1]),
                            )
                        )
                        contour_index += 1
            # Half-bead spacing also fills the final narrow region where two
            # inward offset fronts meet; full-bead offsets can leave a gap.
            inset += width * 0.475
        if layer % 25 == 0 or layer == max(layers):
            print(f"  Sliced layer {layer + 1}/{len(layers)}: {len(tasks)} tasks", flush=True)
    if not tasks:
        raise ValueError("Target produced no printable contours at the configured bead width")
    return tasks

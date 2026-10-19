"""Local inter-layer dependencies for immutable deposition polylines.

This is a geometric scheduling model, not a thermal or mechanical support
certification. Adjacent bead footprints define predecessors. Where the entire
upper centerline is not covered by lower footprints, retain a conservative
barrier to all tasks in the preceding layer and report that fallback explicitly.
"""

from collections import defaultdict
from dataclasses import dataclass

from shapely import LineString, STRtree, union_all


@dataclass(frozen=True)
class Dependencies:
    predecessors: tuple
    successors: tuple
    fallback_tasks: tuple

    def report(self, tasks):
        return dict(
            policy="adjacent_bead_overlap_with_uncovered_centerline_barrier_v4",
            task_count=len(tasks),
            dependency_count=sum(map(len, self.predecessors)),
            fallback_task_count=len(self.fallback_tasks),
            fallback_task_ids=[tasks[i].task_id for i in self.fallback_tasks],
            dependencies={tasks[i].task_id: [tasks[j].task_id for j in parents]
                          for i, parents in enumerate(self.predecessors)},
        )


def build_dependencies(tasks, config):
    """Require all touching lower bead footprints, without a global layer lock.

    Task boundaries and original geometry remain unchanged. A task is atomic:
    every predecessor must finish deposition and arc-off before it can start.
    Missing intermediate layers fail explicitly instead of treating an upper
    layer as a new build plate. The first supplied layer is the input's base.
    """
    layers = defaultdict(list)
    for i, task in enumerate(tasks):
        layers[task.layer].append(i)
    predecessors = [() for _ in tasks]
    successors = [[] for _ in tasks]
    fallbacks = []
    previous_ids, previous_shapes, previous_layer = None, None, None
    radius = config.process.bead_width_mm / 2.0
    if radius <= 0:
        raise ValueError("Positive bead width is required for local support dependencies")
    resolution = config.shape_validation.polygon_buffer_resolution
    epsilon = config.collision.geometry_epsilon_mm
    for layer, indices in sorted(layers.items()):
        lines = [LineString([p[:2] for p in tasks[i].points]) for i in indices]
        shapes = [line.buffer(radius, quad_segs=resolution) for line in lines]
        if previous_ids is not None:
            if layer != previous_layer + 1:
                raise ValueError(f"Missing support layer below layer {layer}")
            tree = STRtree(previous_shapes)
            for i, line, shape in zip(indices, lines, shapes):
                hits = sorted(int(j) for j in tree.query(shape, predicate="intersects"))
                covered = (bool(hits) and union_all([previous_shapes[j] for j in hits])
                           .buffer(epsilon).covers(line))
                if covered:
                    parents = tuple(previous_ids[j] for j in hits)
                else:
                    # Preserve the old ordering for ambiguous/overhanging paths.
                    # This fallback does not claim to repair unsupported geometry.
                    parents = tuple(previous_ids)
                    fallbacks.append(i)
                predecessors[i] = parents
                for parent in parents:
                    successors[parent].append(i)
        previous_ids, previous_shapes, previous_layer = indices, shapes, layer
    return Dependencies(tuple(predecessors), tuple(map(tuple, successors)), tuple(fallbacks))


def verify_precedence(tasks, assignments, dependencies, arc_off_s):
    """Check all required finishes against actual rendered task starts."""
    by_id = {row["task_id"]: row for row in assignments}
    if len(by_id) != len(tasks) or set(by_id) != {t.task_id for t in tasks}:
        raise ValueError("Dependency verification requires exactly one assignment per task")
    for i, parents in enumerate(dependencies.predecessors):
        start = by_id[tasks[i].task_id]["job_start_s"]
        for j in parents:
            release = by_id[tasks[j].task_id]["finish_s"] + arc_off_s
            if start < release - 1e-6:
                raise ValueError(f"Support precedence violated: {tasks[j].task_id} -> {tasks[i].task_id}")
    return dict(status="PASS", checked_dependencies=sum(map(len, dependencies.predecessors)))

"""Min-Min scheduling of generated paths under local support and collision constraints."""
from __future__ import annotations

from . import common as c
from .checkpoint import restore, snapshot
from .common import RobotState, Task
from .precedence import build_dependencies, verify_precedence
from .scheduler import ScheduleBuilder, conflict_table, prepare, render
from .selection import select_ready

ALGORITHM_NAME = "min_min"


def schedule(
    tasks: list[Task], *, progress=None, diagnostics=None, prune=True,
    resume_state=None, save_state=None
) -> tuple[dict[int, RobotState], list[dict[str, object]]]:
    """Construct a deterministic schedule without modifying deposition paths.

    For each remaining task, select its earliest-finishing feasible robot.
    Commit the task with the smallest such completion time, then recompute
    candidates against the updated schedule. Tasks from different layers compete
    once their local support predecessors have been scheduled. Release times
    enforce predecessor deposition and arc-off completion in actual time.
    """
    ordered, routes, costs = prepare(tasks)
    dependencies = build_dependencies(ordered, c.CONFIG)
    builder = ScheduleBuilder(conflict_table(routes, ordered))
    genes = [None] * len(ordered)
    pending = [len(parents) for parents in dependencies.predecessors]
    ready = {i for i, count in enumerate(pending) if count == 0}
    releases = [0.0] * len(ordered)
    committed = 0
    counters = dict(candidate_pairs=0, evaluated_pairs=0, pruned_pairs=0)
    recoveries = []
    if resume_state is not None:
        builder = restore(builder.context, resume_state)
        counters.update(resume_state.get("counters", {}))
        recoveries = list(resume_state.get("recoveries", []))
        for job in builder.jobs:
            i, rid = job.task_index, job.robot_id
            if i not in ready or rid not in costs[i] or job.start_s < releases[i] - 1e-6:
                raise ValueError("Checkpoint violates task eligibility or predecessor release")
            ready.remove(i)
            genes[i] = rid
            for child in dependencies.successors[i]:
                releases[child] = max(releases[child], job.deposit_finish_s +
                                      (c.CONFIG.process.arc_off_time_s or 0.0))
                pending[child] -= 1
                if pending[child] == 0:
                    ready.add(child)
        committed = len(builder.priority)
        print(f"[checkpoint] Restored {committed}/{len(ordered)} assigned tasks", flush=True)
    while ready:
        best = select_ready(builder, ready, costs, releases, counters, prune=prune)
        if best is None:
            for parked in builder.parking_alternatives():
                choice = select_ready(parked, ready, costs, releases, counters, prune=prune)
                if choice is not None and (best is None or choice[:3] < best[:3]):
                    best = choice
        if best is None:
            for parked in builder.waiting_parking_alternatives():
                choice = select_ready(parked, ready, costs, releases, counters, prune=prune)
                if choice is not None and (best is None or choice[:3] < best[:3]):
                    best = choice
            if best is not None:
                recoveries.append(dict(after_tasks=committed, method="wait_then_egress"))
                print(f"[recovery] {committed} tasks | Safe wait and blocker egress", flush=True)
        if best is None:
            if save_state is not None:
                save_state(snapshot(builder, counters, recoveries))
            raise ValueError("No collision-free continuation after immediate and delayed blocker egress")
        _, i, rid, builder = best
        genes[i] = rid
        ready.remove(i)
        release = builder.work_available(rid)
        for child in dependencies.successors[i]:
            releases[child] = max(releases[child], release)
            pending[child] -= 1
            if pending[child] == 0:
                ready.add(child)
        committed += 1
        if save_state is not None and (committed % 100 == 0 or not ready):
            save_state(snapshot(builder, counters, recoveries))
        if progress is not None and (committed % 10 == 0 or not ready):
            progress(ordered[i].layer, committed, len(ordered))
    if committed != len(ordered):
        raise ValueError("Unscheduled tasks remain in the dependency graph")
    states, assignments = render(ordered, routes, tuple(genes), jobs=builder.jobs,
                                priority=builder.priority)
    verification = verify_precedence(ordered, assignments, dependencies,
                                     c.CONFIG.process.arc_off_time_s or 0.0)
    if diagnostics is not None:
        diagnostics["recoveries"] = recoveries
        diagnostics["selection"] = dict(policy="lower_bound_pruning" if prune else "reference",
                                        **counters)
        diagnostics.update(dependencies.report(ordered))
        diagnostics["verification"] = verification
        layer_finishes = {}
        for row in assignments:
            layer_finishes[row["layer"]] = max(layer_finishes.get(row["layer"], 0.0),
                row["finish_s"] + (c.CONFIG.process.arc_off_time_s or 0.0))
        diagnostics["tasks_started_before_previous_layer_finished"] = sum(
            row["deposit_start_s"] < layer_finishes.get(row["layer"] - 1, 0.0) - 1e-6
            for row in assignments)
    return states, assignments



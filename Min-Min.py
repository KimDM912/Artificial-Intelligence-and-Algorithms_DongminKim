"""Layer-wise Min-Min scheduling for collision-constrained three-robot WAAM.

Candidate completion times include travel, arc setup, deposition, collision
delays, and a provisional home return. Safe endpoint-to-endpoint connections
replace provisional returns when subsequent tasks are committed.
"""

from __future__ import annotations

from common.common import RobotState, Task
from common.scheduler import ScheduleBuilder, best_eft, conflict_table, prepare, render

ALGORITHM_NAME = "min_min"


def schedule(
    tasks: list[Task], *, progress=None
) -> tuple[dict[int, RobotState], list[dict[str, object]]]:
    """Construct a deterministic schedule without modifying deposition paths.

    For each remaining task, select its earliest-finishing feasible robot.
    Commit the task with the smallest such completion time, then recompute
    candidates against the updated schedule. Layers are processed in order.
    """
    ordered, routes, costs = prepare(tasks)
    builder = ScheduleBuilder(conflict_table(routes, ordered))
    genes = [None] * len(ordered)
    committed = 0
    for layer in sorted({task.layer for task in ordered}):
        remaining = {i for i, task in enumerate(ordered) if task.layer == layer}
        while remaining:
            best_for_task = []
            for i in remaining:
                rid, candidate = best_eft(builder, i, costs[i])
                finish = candidate.by_robot[rid][-1].finish_s
                best_for_task.append((finish, i, rid, candidate))
            # Task indices follow (layer, task_id); robot ties are resolved in best_eft.
            _, i, rid, builder = min(best_for_task, key=lambda row: row[:3])
            genes[i] = rid
            remaining.remove(i)
            committed += 1
            if progress is not None and (committed % 10 == 0 or not remaining):
                progress(layer, committed, len(ordered))
    return render(ordered, routes, tuple(genes), jobs=builder.jobs, priority=builder.priority)


if __name__ == "__main__":
    from common.runner import run

    raise SystemExit(run(schedule))

"""JSON scheduling snapshots without executable objects or collision caches."""

import hashlib
import json
import math

from .scheduler import ScheduleBuilder, ScheduledJob


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode("utf-8")).hexdigest()


def snapshot(builder, counters, recoveries):
    return dict(priority=list(builder.priority), counters=dict(counters), recoveries=list(recoveries),
                jobs=[dict(task_index=j.task_index, robot_id=j.robot_id, start_s=j.start_s,
                           deposit_start_s=j.deposit_start_s, deposit_finish_s=j.deposit_finish_s,
                           finish_s=j.finish_s, plan_key=list(j.plan.key[:5]),
                           retreat_wait_s=j.plan.retreat_wait_s) for j in builder.jobs])


def restore(context, state):
    priority = state["priority"]
    if len(priority) != len(set(priority)) or len(priority) != len(state["jobs"]):
        raise ValueError("Invalid checkpoint task sequence")
    indexed = {}
    for row in state["jobs"]:
        i, rid = row["task_index"], row["robot_id"]
        if i in indexed or type(i) is not int or not 0 <= i < len(context.ordered):
            raise ValueError("Invalid checkpoint task index")
        key = row["plan_key"]
        if len(key) < 5 or key[0] != i or key[1] != rid:
            raise ValueError("Invalid checkpoint route identity")
        if key[2] >= 0 and (key[2] not in indexed or indexed[key[2]].robot_id != rid):
            raise ValueError("Checkpoint route source is not an assigned task of this robot")
        plan = context.transition(i, rid, key[2], return_home=bool(key[3]), approach_mode=key[4])
        if plan is None:
            raise ValueError("Checkpoint route is unreachable")
        delay = row.get("retreat_wait_s", 0.0)
        plan = context.delayed_retreat(plan, delay)
        values = [row[k] for k in ("start_s", "deposit_start_s", "deposit_finish_s", "finish_s")]
        if not all(math.isfinite(v) and v >= 0 for v in values) or values != sorted(values):
            raise ValueError("Invalid checkpoint times")
        if not math.isclose(row["finish_s"], row["start_s"] + plan.duration_s, abs_tol=1e-5, rel_tol=0):
            raise ValueError("Checkpoint route duration mismatch")
        indexed[i] = ScheduledJob(i, rid, *values, plan)
    if set(indexed) != set(priority) or list(indexed) != priority:
        raise ValueError("Checkpoint jobs do not match their commitment order")
    builder = ScheduleBuilder(context)
    for i in priority:
        job = indexed[i]
        old = builder.by_robot[job.robot_id]
        if old and job.start_s < old[-1].finish_s - 1e-6:
            raise ValueError("Overlapping checkpoint jobs")
        builder.by_robot[job.robot_id] = (*old, job)
    builder.priority = tuple(priority)
    return builder

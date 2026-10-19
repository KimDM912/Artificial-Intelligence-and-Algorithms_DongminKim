"""Min-Min selection with conservative lower-bound pruning, not stale EFT reuse."""

import math

from . import common as c
from .scheduler import best_eft


def finish_lower_bound(builder, i, rid, processing_s, release_s):
    """Earliest possible finish without collisions or a new optional retreat.

    A previous reserved retreat may have finished before the release time. Its
    home-departure alternative must therefore be considered separately from a
    cancelled-retreat endpoint connection. No old feasible completion time is
    assumed to remain valid after another job is committed.
    """
    jobs = builder.by_robot[rid]
    if not jobs:
        return release_s + processing_s  # Free initial placement is permitted.
    previous = jobs[-1]
    ready = builder.work_available(rid)
    start = builder.context.ordered[i].start
    endpoint = builder.context.ordered[previous.task_index].end
    speed = c.CONFIG.process.travel_speed_mm_s
    earliest = max(release_s, ready) + math.dist(endpoint, start) / speed
    home = tuple(c.ROBOTS[rid]["home"])
    if previous.plan.retreat[-1] == home:
        earliest = min(earliest, max(release_s, previous.finish_s)
                       + math.dist(home, start) / speed)
    return earliest + processing_s


def strictly_worse(lower, incumbent):
    """Never prune ties; numerical slack affects pruning only, not the schedule."""
    if not math.isfinite(incumbent):
        return False
    slack = max(1e-9, 64 * math.ulp(max(abs(lower), abs(incumbent))))
    return lower > incumbent + slack


def select_ready(builder, ready, costs, releases, counters, *, prune=True):
    """Preserve robot ties (finish, start, ID) and task ties (finish, index, ID)."""
    best = None
    if not prune:
        for i in sorted(ready):
            counters["candidate_pairs"] += len(costs[i])
            counters["evaluated_pairs"] += len(costs[i])
            rid, candidate = best_eft(builder, i, costs[i], release_s=releases[i])
            if candidate is not None:
                choice = (candidate.by_robot[rid][-1].finish_s, i, rid, candidate)
                if best is None or choice[:3] < best[:3]:
                    best = choice
        return best

    ranked = []
    for i in ready:
        bounds = sorted((finish_lower_bound(builder, i, rid, cost, releases[i]), rid)
                        for rid, cost in costs[i].items())
        ranked.append((bounds[0][0], i, bounds))
        counters["candidate_pairs"] += len(bounds)
    for lower, i, bounds in sorted(ranked):
        if best is not None and strictly_worse(lower, best[0]):
            counters["pruned_pairs"] += len(bounds)
            continue
        robot_best = None
        for bound, rid in bounds:
            cutoff = min(best[0] if best else math.inf,
                         robot_best[0] if robot_best else math.inf)
            if strictly_worse(bound, cutoff):
                counters["pruned_pairs"] += 1
                continue
            counters["evaluated_pairs"] += 1
            candidate = builder.append(i, rid, release_s=releases[i])
            if candidate is None:
                continue
            job = candidate.by_robot[rid][-1]
            row = (job.finish_s, job.start_s, rid, candidate)
            if robot_best is None or row[:3] < robot_best[:3]:
                robot_best = row
        if robot_best is not None:
            finish, _, rid, candidate = robot_best
            choice = (finish, i, rid, candidate)
            if best is None or choice[:3] < best[:3]:
                best = choice
    return best

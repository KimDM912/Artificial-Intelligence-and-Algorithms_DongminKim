"""The four experiment metrics, using a common [0, makespan] horizon."""

import math

METRIC_FIELDS = ["makespan_s", "computation_time_s"] + [
    field for rid in (1, 2, 3) for field in (f"robot_{rid}_utilization", f"robot_{rid}_idle_time_s")
]


def evaluate_metrics(states, assignments, computation_time_s):
    if set(states) != {1, 2, 3}:
        raise ValueError("Metrics require robots 1, 2, 3")
    makespan = max(state.available_s for state in states.values())
    if not math.isfinite(makespan) or makespan < 0:
        raise ValueError("Invalid makespan")
    if not math.isfinite(computation_time_s) or computation_time_s < 0:
        raise ValueError("Invalid computation time")
    result = dict(makespan_s=makespan, computation_time_s=computation_time_s)
    for rid in sorted(states):
        intervals = sorted(
            (float(a["job_start_s"]), float(a["job_finish_s"]))
            for a in assignments
            if a["robot_id"] == rid
        )
        previous = 0.0
        busy = 0.0
        for start, finish in intervals:
            if (
                not math.isfinite(start)
                or not math.isfinite(finish)
                or start < previous - 1e-6
                or finish < start
                or finish > makespan + 1e-6
            ):
                raise ValueError(f"Invalid/overlapping occupancy for robot {rid}")
            busy += finish - start
            previous = finish
        # Busy includes approach, deposition, arc setup and return home.
        result[f"robot_{rid}_utilization"] = min(1.0, busy / makespan) if makespan else 0.0
        result[f"robot_{rid}_idle_time_s"] = max(0.0, makespan - busy)
    return result


def parallelism_diagnostics(assignments, makespan_s):
    """Explain concurrency separately from the four comparison metrics.

    Occupancy includes travel/setup; deposition counts only active printing.
    Intervals are half-open, so a hand-off at the same instant is not overlap.
    """
    result = {}
    for label, start_key, finish_key in (
        ("occupancy", "job_start_s", "job_finish_s"),
        ("deposition", "deposit_start_s", "finish_s"),
    ):
        events = {0.0: 0, makespan_s: 0}
        for row in assignments:
            start, finish = row[start_key], row[finish_key]
            if finish > start:
                events[start] = events.get(start, 0) + 1
                events[finish] = events.get(finish, 0) - 1
        durations = [0.0] * 4
        active, previous = 0, 0.0
        for time, delta in sorted(events.items()):
            durations[active] += time - previous
            active += delta
            previous = time
        result[label] = dict(
            max_concurrent_robots=max((n for n, dt in enumerate(durations) if dt > 0), default=0),
            two_or_more_robots_s=sum(durations[2:]),
            three_robots_s=durations[3],
            mean_active_robots=(
                sum(n * dt for n, dt in enumerate(durations)) / makespan_s if makespan_s else 0.0
            ),
        )
    return result

"""Shared batch execution. Algorithms expose schedule(tasks) and a unique name."""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from time import perf_counter

from . import common as c


def run(schedule, algorithm_name, argv=None):
    parser = argparse.ArgumentParser(description=f"Run {algorithm_name} (default: all six cases)")
    parser.add_argument("case", nargs="?", type=int, choices=range(1, 7))
    parser.add_argument("--tasks-csv", type=Path,
                        help="Explicit task input; requires a single case")
    args = parser.parse_args(argv)
    if args.tasks_csv is not None and args.case is None:
        parser.error("--tasks-csv requires a case number")
    if not algorithm_name or Path(algorithm_name).name != algorithm_name or algorithm_name in {".", ".."}:
        parser.error("Algorithm name must be a single directory name")

    # Independent of the PowerShell working directory.
    project = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(project))
    output = project / "experiment_algorighm" / "result" / algorithm_name
    output.mkdir(parents=True, exist_ok=True)
    cases = [args.case] if args.case is not None else list(range(1, 7))
    summaries = []
    failed = False
    for case in cases:
        algorithm_time_s = ""
        task_generation_time_s = ""
        case_started = perf_counter()
        try:
            config = project / "tests" / f"{case:02d}" / "config.yaml"
            c.load_config(config)
            generation_started = perf_counter()
            try:
                tasks = (c.read_tasks(args.tasks_csv) if args.tasks_csv is not None
                         else c.build_benchmark_tasks(case))
            finally:
                task_generation_time_s = perf_counter() - generation_started
            if not tasks:
                raise ValueError("No tasks were generated")
            started = perf_counter()
            try:
                states, assignments = schedule(tasks)
            finally:
                algorithm_time_s = perf_counter() - started
            trajectory = output / f"trajectory_{case:02d}.csv"
            c.write_trajectory(states, trajectory)
            c.write_assignments(assignments, output / f"assignments_{case:02d}.csv")
            makespan = max(s.available_s for s in states.values())
            summaries.append(dict(case=case, status="OK", makespan_s=makespan,
                                  algorithm_time_s=algorithm_time_s,
                                  task_generation_time_s=task_generation_time_s,
                                  total_time_s=perf_counter() - case_started,
                                  tasks=len(tasks), error=""))
            print(f"[{case:02d}/06] Makespan: {makespan:.3f} s | "
                  f"Algorithm time: {algorithm_time_s:.6f} s | {trajectory}", flush=True)
        except Exception as exc:
            failed = True
            summaries.append(dict(case=case, status="ERROR", makespan_s="",
                                  algorithm_time_s=algorithm_time_s,
                                  task_generation_time_s=task_generation_time_s,
                                  total_time_s=perf_counter() - case_started,
                                  tasks="", error=str(exc)))
            print(f"[{case:02d}/06] ERROR: {exc}", file=sys.stderr, flush=True)
    with (output / "run_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["case", "status", "makespan_s",
                                             "algorithm_time_s", "task_generation_time_s",
                                             "total_time_s", "tasks", "error"])
        writer.writeheader()
        writer.writerows(summaries)
    return 1 if failed else 0

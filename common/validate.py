"""Validate saved algorithm trajectories against their matching benchmark cases."""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
RESULTS = PROJECT / "experiment_algorighm" / "result"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("algorithm", help="Saved result directory name, e.g. lpt_eft")
    parser.add_argument("--mode", choices=("run", "check", "ui"), default="run")
    parser.add_argument("--case", type=int, choices=range(1, 7), help="Only this case (1-6)")
    args = parser.parse_args(argv)
    if (not args.algorithm or args.algorithm in {".", ".."}
            or any(char in args.algorithm for char in '/\\:')):
        parser.error("algorithm must be a single directory name")
    source = RESULTS / args.algorithm
    if not source.is_dir():
        parser.error(f"Algorithm results not found: {source}")
    if args.mode == "ui" and args.case is None:
        parser.error("--mode ui requires --case 1 (or another case number)")

    # Each invocation keeps its own input snapshot and validation outputs.
    root = source / "validation"
    root.mkdir(parents=True, exist_ok=True)
    batch = Path(tempfile.mkdtemp(
        prefix=f"{datetime.now():%Y%m%d_%H%M%S}_{args.mode}_", dir=root,
    ))
    env = os.environ.copy()
    env["PYTHONPATH"] = str(PROJECT / "src") + os.pathsep + env.get("PYTHONPATH", "")
    cases = [args.case] if args.case is not None else list(range(1, 7))
    rows = []
    print(f"Results: {batch}", flush=True)
    for case in cases:
        job = batch / f"{case:02d}"
        row = dict(case=f"{case:02d}", mode=args.mode, status="ERROR", exit_code=2,
                   job_dir=str(job), error="")
        print(f"\n=== {args.algorithm} / {case:02d} / {args.mode} ===", flush=True)
        try:
            inputs = {
                "config.yaml": PROJECT / "tests" / f"{case:02d}" / "config.yaml",
                "target.stl": PROJECT / "tests" / f"{case:02d}" / "target.stl",
                "trajectory.csv": source / f"trajectory_{case:02d}.csv",
            }
            for path in inputs.values():
                if not path.is_file():
                    raise FileNotFoundError(f"Required input not found: {path}")
            job.mkdir()
            for name, path in inputs.items():
                shutil.copy2(path, job / name)
            completed = subprocess.run(
                [sys.executable, "-m", "waam_validator.cli", args.mode, str(job)],
                cwd=PROJECT, env=env, check=False,
            )
            row["exit_code"] = completed.returncode
            row["status"] = (
                "PASS" if completed.returncode == 0 else
                "FAIL" if completed.returncode == 1 else "ERROR"
            )
        except OSError as exc:
            row["error"] = str(exc)
            print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        rows.append(row)
        # Save after each case so completed results survive an interrupted batch.
        with (batch / "validation_summary.csv").open(
            "w", newline="", encoding="utf-8-sig",
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerows(rows)

    print("\n=== Batch summary ===", flush=True)
    for row in rows:
        print(f"{row['case']}: {row['status']} (exit {row['exit_code']})", flush=True)
    print(f"Summary: {batch / 'validation_summary.csv'}", flush=True)
    return 1 if any(row["exit_code"] != 0 for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())

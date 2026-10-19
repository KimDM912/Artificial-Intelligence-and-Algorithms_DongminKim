# Min-Min Scheduling for Multi-Robot WAAM

A deterministic pipeline for deposition-path generation and task allocation to three robots in Wire Arc Additive Manufacturing (WAAM).

Given a target STL and a WAAM Validator configuration, the program generates deposition paths, reduces their length subject to geometry checks, assigns the resulting tasks using Min-Min, and validates the complete trajectory. The same procedure is used for the included cases and new input geometries; no historical experiment results are required.

## Repository Structure

```text
.
├── Min-Min.py              # Generation, scheduling, and validation entry point
├── View_Result.py          # Open a completed run in the Validator UI
├── requirements.txt        # Pinned WAAM Validator dependency
├── common/                 # Path generation, scheduling, and validation utilities
├── inputs/
│   ├── 01/                 # config.yaml and target.stl
│   ├── ...
│   └── 06/
└── tests/
```

## Installation

Use Python 3.11 and an activated Python environment. Git must be available to install the pinned dependency.

Run the following commands from the directory containing `Min-Min.py`:

```powershell
python -m pip install -r requirements.txt
```

The dependency is [WAAM Validator](https://github.com/Ha-An/WAAM_Validator), pinned to commit `283b1beed324aaba717fe76bd06449e9001dac41` in `requirements.txt`.

## Method

### Deposition-Path Generation

1. Slice the STL using the layer height and coordinate settings from `config.yaml`.
2. Generate contour and hatch candidates for each printable layer, including candidates that remove nearly overlapping contour passes.
3. Check robot reachability and deposited geometry. Select a feasible candidate using deposition time plus arc-on/off time as a processing-cost estimate.
4. Split long paths into tasks and remove numerically unresolved path segments.
5. Shorten both ends of each task while screening the resulting layer geometry against the IoU target and the configured shape criteria.

The generation cost estimate does not include the final scheduling effects of travel, collisions, or robot workload. Those effects are evaluated during allocation.

### Min-Min Allocation

For each ready, unassigned task, estimate its completion time on every eligible robot. The estimates account for the robot's scheduled work, travel, arc operations, deposition, support dependencies, and collision-induced waiting.

Select the earliest-finishing robot for each task. Among these task–robot pairs, commit the pair with the earliest completion time. Update the schedule and repeat until every task is assigned.

Local support dependencies determine when upper-layer tasks become ready. Collision checks include scheduled motion and stationary robots. The scheduler considers direct and safe-height connections, waiting, and optional robot egress. Candidate pruning avoids evaluations that cannot improve the current selection.

### Full Validation and Fallback

Geometry screening alone is not acceptance. The complete trajectory must pass the official Validator, including its trajectory, speed, reachability, collision, and shape checks.

If scheduling or full validation of the shortened paths fails, the pipeline retries with the generated paths before endpoint shortening. A layer whose shortening fails its geometry screen already retains its original generated paths.

The first fully validated candidate is published. The unshortened candidate is a failure fallback; it is not also scheduled to compare makespans after the shortened candidate passes. The method does not guarantee a globally optimal makespan or success for every geometry.

## Inputs and Default Settings

An input directory must contain:

| File | Purpose |
|---|---|
| `config.yaml` | Robot geometry, process settings, collision settings, and validation criteria |
| `target.stl` | Target geometry |

The configuration must define three robots with IDs `1`, `2`, and `3`. Both input files are copied unchanged into the result bundle. Deposition paths are generated from the STL; a pre-existing `deposition_paths.json` in the input directory is not used.

| Option or setting | Default | Meaning |
|---|---:|---|
| `--iou-floor` | `0.90` | Requested minimum IoU for path selection |
| Geometry-screening margin | `0.001` | Added to the effective IoU floor, capped at `1.0` |
| `--max-task-length` | `600.0` mm | Path-splitting parameter |
| Endpoint shortening | Enabled | Disable with `--no-trim` |

The effective IoU floor is the maximum of the requested floor and the configured overall and per-layer IoU thresholds. The screening margin is then added. With the included thresholds and default settings, the screening target is **0.901 (90.1%)**. This is a minimum acceptance target, not an exact output IoU.

These generation parameters do not modify `config.yaml`. Coverage, overfill, layer failure limits, speeds, and collision settings remain subject to the supplied configuration and full Validator.

## Run a Case

Run Task 1 with the default IoU floor of `0.90`:

```powershell
python .\Min-Min.py --input-dir .\inputs\01 --results-dir .\results\case_01
```

Run Task 4:

```powershell
python .\Min-Min.py --input-dir .\inputs\04 --results-dir .\results\case_04
```

Run Tasks 1–6 sequentially in PowerShell:

```powershell
1..6 | ForEach-Object { $caseId = "{0:D2}" -f $_; python .\Min-Min.py --input-dir ".\inputs\$caseId" --results-dir ".\results\case_$caseId"; if ($LASTEXITCODE -ne 0) { throw "Case $caseId failed; inspect its run records." } }
```

Run a new geometry by placing its configuration and STL in another directory:

```powershell
python .\Min-Min.py --input-dir "C:\data\new_case" --results-dir .\results\new_case
```

Optional controls:

```powershell
# Override the IoU floor.
python .\Min-Min.py --input-dir .\inputs\01 --results-dir .\results\case_01 --iou-floor 0.92

# Generate paths and screen geometry without scheduling or full validation.
python .\Min-Min.py --input-dir .\inputs\01 --results-dir .\results\case_01 --prepare-only

# Keep generated paths without endpoint shortening.
python .\Min-Min.py --input-dir .\inputs\01 --results-dir .\results\case_01 --no-trim
```

## Outputs and Timing

Each execution creates a timestamped run directory:

```text
results/case_01/<timestamp>/
├── input/                 # Unchanged configuration and target
├── source/                # Source snapshot
├── prepared/              # Generated paths before/after shortening
├── allocation/            # Schedules, checkpoints, metrics, and Validator reports
├── trajectory/            # Created only after full Validator PASS
│   ├── config.yaml
│   ├── target.stl
│   └── trajectory.csv
├── generation.json
├── geometry_screen.json
├── attempts.json
└── summary.json
```

`summary.json` records the selected variant, makespan, total computation time, generation time, allocation time, and Validator time. Total computation time includes measured generation, allocation/export work, and any failed candidate attempt before fallback. Full Validator execution time is recorded separately and excluded from total computation time.

JSON timing values are in seconds. Console makespan and elapsed-time output uses `DD:HH:MM:SS`. Detailed robot metrics and validation results are stored under the selected allocation attempt.

## Validator UI

After Task 1 completes successfully:

```powershell
python .\View_Result.py --case 1
```

For Task 4:

```powershell
python .\View_Result.py --case 4
```

The helper selects the latest run under `results/case_XX`, verifies its PASS record and published input hashes, and launches the official UI at `http://127.0.0.1:8050`. It reports an error if the latest run is incomplete or failed, rather than displaying an older result.

For a custom result directory or a particular run, replace `<timestamp>` with the actual directory name:

```powershell
python .\View_Result.py --session ".\results\new_case\<timestamp>"
```

The three-file bundle can also be opened directly without the experiment records:

```powershell
python -m waam_validator.cli ui ".\results\case_01\<timestamp>\trajectory"
```

## Resume a Run

Reuse a complete path-generation stage and resume allocation:

```powershell
python .\Min-Min.py --resume-session ".\results\case_01\<timestamp>"
```

Resume verifies the code, environment, and input hashes. An interrupted allocation marked `RUNNING` can continue from its saved scheduling checkpoint. Verified PASS results are reused; recorded failed candidates advance to the fallback. Incomplete path generation requires a new run. Changes to code, environment, or inputs also require a new session.

## Sharing Results

For direct evaluation, provide all three files from a successful run's `trajectory/` directory together: **`config.yaml`, `target.stl`, and `trajectory.csv`**.

`results/` is ignored by Git by default. To distribute precomputed outputs without uploading every experiment record, copy each accepted three-file bundle into a tracked directory such as `trajectory/01/`. Evaluators can then run:

```powershell
python -m waam_validator.cli ui .\trajectory\01
```

The source code, `common/`, `requirements.txt`, and the corresponding input files are sufficient to reproduce a run. Precomputed bundles allow evaluation without rerunning path generation and allocation.

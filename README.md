# Min-Min Scheduling for Multi-Robot WAAM

A deterministic pipeline for deposition-path generation and task allocation to three robots in Wire Arc Additive Manufacturing (WAAM).

Given a target STL and a WAAM Validator configuration, the program generates deposition paths, refines them subject to geometry checks, assigns the resulting tasks using Min-Min, and validates the complete trajectory. The same procedure applies to the included cases and new input geometries.

**For evaluation:** precomputed, full-Validator PASS bundles for Tasks 1-6 are included in `result/01/` through `result/06/`. Each directory contains `config.yaml`, `target.stl`, and `trajectory.csv`. After installing the dependency, evaluators can open these bundles directly; running `Min-Min.py` is only necessary to regenerate trajectories or process a new geometry.

## Repository Structure

```text
Min-Min.py           Generation, scheduling, and validation entry point
View_Result.py       Open a completed run in the Validator UI
Export_Result.py     Collect validated evaluation bundles
requirements.txt     Pinned WAAM Validator dependency
common/              Path generation, scheduling, and validation utilities
inputs/01/ ... 06/   Benchmark configurations and target STL files
result/01/ ... 06/   Validated config.yaml, target.stl, and trajectory.csv
tests/               Automated checks
```

## Installation

Use Python 3.11 and an activated Python environment. Git must be available to install the pinned dependency.

Run commands from the directory containing `Min-Min.py`:

```powershell
python -m pip install -r requirements.txt
```

The dependency is [WAAM Validator](https://github.com/Ha-An/WAAM_Validator), pinned in `requirements.txt`.

## Method

### Deposition-Path Generation

1. Slice the STL using the layer height and coordinate settings from `config.yaml`.
2. Generate contour and hatch candidates for each printable layer, including candidates that remove nearly overlapping contour passes.
3. Check robot reachability and deposited geometry. Select a feasible candidate using deposition time plus arc-on/off time as a processing-cost estimate.
4. Split long paths into tasks and remove numerically unresolved path segments.
5. Shorten path endpoints subject to geometry screening before allocation.

Travel, collisions, and workload balance are evaluated during scheduling. The path-generation cost estimate is not the final makespan.

### Min-Min Allocation

For each ready, unassigned task, estimate its completion time on every eligible robot. These estimates account for scheduled work, travel, arc operations, deposition, support dependencies, and collision-induced waiting.

Select the earliest-finishing robot for each task. Among these task-robot pairs, commit the pair with the earliest completion time. Update the schedule and repeat until every task is assigned.

Local support dependencies determine when upper-layer tasks become ready. Collision checks include scheduled motion and stationary robots. The scheduler considers direct and safe-height connections, waiting, and optional robot egress. Candidate pruning avoids evaluations that cannot improve the current selection.

### Validation

The complete trajectory must pass the official Validator, including trajectory, speed, reachability, collision, and shape checks. Geometry screening alone is not acceptance.

If scheduling or full validation of the shortened paths fails, the pipeline retries with the generated paths before endpoint shortening. Only a fully validated trajectory is published. The method does not guarantee a globally optimal makespan or success for every geometry.

## Inputs

An input directory must contain:

| File | Purpose |
|---|---|
| `config.yaml` | Robot geometry, process settings, collision settings, and validation criteria |
| `target.stl` | Target geometry |

The configuration must define three robots with IDs `1`, `2`, and `3`. Both files are copied unchanged into the validated output bundle. Deposition paths are generated from the STL; a pre-existing deposition-path file is not required or used.

## Execution

Run Task 1:

```powershell
python .\Min-Min.py --input-dir .\inputs\01
```

Run Task 4:

```powershell
python .\Min-Min.py --input-dir .\inputs\04
```

Run Tasks 1-6 sequentially in PowerShell:

```powershell
1..6 | ForEach-Object { $caseId = "{0:D2}" -f $_; python .\Min-Min.py --input-dir ".\inputs\$caseId"; if ($LASTEXITCODE -ne 0) { throw "Case $caseId failed." } }
```

Run a new geometry using a directory containing its configuration and STL:

```powershell
python .\Min-Min.py --input-dir "C:\data\new_case"
```

Successful execution prints the validated input directory. Console makespan and computation-time output uses `DD:HH:MM:SS`. Computation time includes measured generation, allocation/export work, and any failed candidate attempt before fallback. Full Validator execution time is recorded separately.

## Validator UI

Run the command for the desired task from the directory containing `Min-Min.py`:

| Task | Validator UI command |
|---|---|
| 1 | `python -m waam_validator.cli ui .\result\01` |
| 2 | `python -m waam_validator.cli ui .\result\02` |
| 3 | `python -m waam_validator.cli ui .\result\03` |
| 4 | `python -m waam_validator.cli ui .\result\04` |
| 5 | `python -m waam_validator.cli ui .\result\05` |
| 6 | `python -m waam_validator.cli ui .\result\06` |

For example, open Task 1 without regenerating its trajectory:

```powershell
python -m waam_validator.cli ui .\result\01
```

Alternatively, use the helper:

```powershell
python .\View_Result.py --case 1
```

The UI is available at `http://127.0.0.1:8050` by default. No experiment logs are needed to evaluate a three-file bundle.

## Evaluation Files

Each task has a separate evaluation bundle:

```text
result/
  01/
    config.yaml
    target.stl
    trajectory.csv
  02/
    config.yaml
    target.stl
    trajectory.csv
  ...
  06/
    config.yaml
    target.stl
    trajectory.csv
```

Include the `result` directory when sharing this repository for direct evaluation.

| File | Source |
|---|---|
| `config.yaml` | Unchanged input configuration |
| `target.stl` | Unchanged input target |
| `trajectory.csv` | Generated and scheduled robot trajectory |

The trajectory contains robot IDs, waypoint times, TCP coordinates, and travel/deposition/wait modes. Evaluators can open this three-file bundle directly with the official Validator without rerunning the algorithm.

Successful executions automatically publish a bundle to `result/<input-directory-name>/`. Use `--output-dir` to choose a different destination. Failed executions do not replace an existing published bundle.

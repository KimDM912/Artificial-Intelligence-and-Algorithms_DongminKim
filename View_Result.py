"""Open the latest completed case in the official WAAM Validator UI."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from common.runner import sha256

PACKAGE = Path(__file__).resolve().parent


def validated_input(session):
    summary_path = session / "summary.json"
    if not summary_path.is_file():
        raise ValueError("The selected run has not finished; summary.json is missing")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("status") != "PASS":
        raise ValueError("The selected run did not PASS full validation")
    # Resolve within this bundle so moving the repository does not break viewing.
    variant = summary["variant"]
    if variant not in ("trimmed", "base"):
        raise ValueError("Unknown path variant")
    name = Path(summary["result_directory"]).name
    result = session / "allocation" / variant / name
    manifest = json.loads((result / "manifest.json").read_text(encoding="utf-8"))
    validation = json.loads((result / "validation.json").read_text(encoding="utf-8"))
    if manifest.get("status") != "PASS" or validation.get("status") != "PASS":
        raise ValueError("Missing full-Validator PASS record")
    inputs = session / "trajectory"
    for filename in ("config.yaml", "target.stl", "trajectory.csv"):
        if sha256(inputs / filename) != manifest["published_input_sha256"][filename]:
            raise ValueError(f"Published Validator input changed: {filename}")
    return inputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--case", type=int, choices=range(1, 7), help="Default: case 1")
    source.add_argument("--session", type=Path, help="Explicit timestamped run directory")
    parser.add_argument("--port", type=int, default=8050)
    parser.add_argument("--check-only", action="store_true", help="Check inputs without launching the UI")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        if args.session:
            session = args.session.resolve()
        else:
            root = PACKAGE / "results" / f"case_{args.case or 1:02d}"
            runs = sorted(p for p in root.glob("*") if p.is_dir() and (p / "run.json").is_file())
            if not runs:
                raise ValueError(f"No runs found in {root}")
            # Do not silently display an older result if the latest run failed.
            session = runs[-1]
        inputs = validated_input(session)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Cannot open result: {exc}\n")
    print(f"Validator inputs: {inputs}", flush=True)
    if args.check_only:
        return 0
    return subprocess.call([sys.executable, "-m", "waam_validator.cli", "ui", str(inputs),
                            "--port", str(args.port)])


if __name__ == "__main__":
    raise SystemExit(main())

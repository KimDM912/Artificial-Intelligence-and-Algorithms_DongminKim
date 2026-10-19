"""Unseen geometry, config-driven generation, fallback and standalone execution."""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from shapely import box
from trimesh.creation import extrude_polygon

from submit_ver5.common import pipeline as p
from submit_ver5.common.provided_paths import build_tasks

ROOT = Path(__file__).resolve().parents[2]


def make_input(folder, *, hole=False, strict=False):
    folder.mkdir()
    config = yaml.safe_load((Path(__file__).parent / "fixtures/config.yaml").read_text())
    config["process"]["deposition_speed_mm_s"] = 7.
    config["process"]["layer_height_mm"] = 3.
    if strict:
        config["shape_validation"]["minimum_overall_iou"] = .95
        config["shape_validation"]["minimum_layer_iou"] = .94
    (folder / "config.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
    geometry = box(-45, -25, 45, 25)
    if hole:
        geometry = geometry.difference(box(-15, -10, 15, 10))
    extrude_polygon(geometry, 6.).export(folder / "target.stl")
    # An obsolete path file must never supply geometry to the new pipeline.
    (folder / "deposition_paths.json").write_text("invalid, deliberately ignored")
    return folder


@pytest.mark.parametrize("hole,strict", [(False, False), (True, True)])
def test_unseen_shape_full_validation_and_resume(tmp_path, hole, strict):
    source = make_input(tmp_path / "unseen", hole=hole, strict=strict)
    before = {name: p.r.sha256(source / name) for name in ("config.yaml", "target.stl")}
    output = tmp_path / "results"
    assert p.main(["--input-dir", str(source), "--results-dir", str(output)]) == 0
    session = next(output.iterdir())
    result = json.loads((session / "summary.json").read_text())
    assert result["status"] == "PASS"
    assert result["computation_time_s"] >= result["allocation_time_s"] + result["generation_time_s"]
    p.verify_pass(Path(result["result_directory"]))
    generation = p.verify_session(session, p.source_hashes())
    assert generation["layer_count"] == 2
    if strict:
        assert generation["settings"]["screened_iou_floor"] == pytest.approx(.951)
        assert generation["overall_iou"] >= .951
    for name, checksum in before.items():
        assert p.r.sha256(source / name) == checksum
        assert p.r.sha256(session / "trajectory" / name) == checksum
    tasks = build_tasks(session / "prepared/base/target.stl", p.c.CONFIG)
    assert {t.layer for t in tasks} == {0, 1}
    assert {t.start[2] for t in tasks} == {1.5, 4.5}
    published = p.r.sha256(session / "trajectory/trajectory.csv")
    viewer = subprocess.run([sys.executable, "-B", str(ROOT / "submit_ver5/View_Result.py"),
        "--session", str(session), "--check-only"], capture_output=True, text=True, timeout=20)
    assert viewer.returncode == 0, viewer.stdout + viewer.stderr
    assert p.main(["--resume-session", str(session)]) == 0
    assert p.r.sha256(session / "trajectory/trajectory.csv") == published
    (session / "prepared/base/config.yaml").write_text("tampered")
    with pytest.raises(ValueError, match="Prepared path input changed"):
        p.main(["--resume-session", str(session)])


def test_rejected_trim_falls_back_and_counts_failed_work(tmp_path, monkeypatch):
    source = make_input(tmp_path / "unseen")
    output = tmp_path / "results"
    validate = p.r.validate
    calls = []

    def reject_first(inputs, report, expected):
        result = validate(inputs, report, expected)
        calls.append(result["status"])
        if len(calls) == 1:
            result.update(status="FAIL", failure_reasons=["Injected rejection for fallback test"])
        return result

    monkeypatch.setattr(p.r, "validate", reject_first)
    assert p.main(["--input-dir", str(source), "--results-dir", str(output)]) == 0
    session = next(output.iterdir())
    result = json.loads((session / "summary.json").read_text())
    attempts = json.loads((session / "attempts.json").read_text())
    assert result["variant"] == "base" and result["status"] == "PASS"
    assert len(calls) == 2 and [r["status"] for r in attempts] == ["ERROR", "PASS"]
    assert result["computation_time_s"] == pytest.approx(result["generation_time_s"] + sum(
        r["attempt_wall_time_s"] - r["validation_time_s"] for r in attempts))


def test_all_rejected_never_publishes(tmp_path, monkeypatch):
    source = make_input(tmp_path / "unseen")
    output = tmp_path / "results"
    monkeypatch.setattr(p.r, "validate", lambda *args: dict(
        status="FAIL", validation_time_s=0., failure_reasons=["Injected rejection"]))
    assert p.main(["--input-dir", str(source), "--results-dir", str(output)]) == 1
    session = next(output.iterdir())
    assert not (session / "trajectory/trajectory.csv").exists()
    result = json.loads((session / "summary.json").read_text())
    assert result["status"] == "ERROR" and result["makespan_s"] is None


def test_package_runs_outside_repository(tmp_path):
    source = make_input(tmp_path / "unseen")
    package = tmp_path / "standalone"
    shutil.copytree(ROOT / "submit_ver5/common", package / "common",
                    ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copyfile(ROOT / "submit_ver5/Min-Min.py", package / "Min-Min.py")
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    result = subprocess.run([sys.executable, "-B", str(package / "Min-Min.py"),
        "--input-dir", str(source), "--results-dir", str(tmp_path / "output")],
        cwd=tmp_path, env=env, capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
    session = next((tmp_path / "output").iterdir())
    assert (session / "trajectory/trajectory.csv").is_file()

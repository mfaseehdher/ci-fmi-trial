import json
from pathlib import Path

import pytest

pytest.importorskip("matplotlib")

import docs.experiments.plot_results as plot_results


def write_csv(path: Path, rows):
    path.write_text("time,Velocity\n" + "".join(f"{t},{v}\n" for t, v in rows), encoding="utf-8")


def test_coupled_run_is_plotted_with_exact_solution(tmp_path, monkeypatch):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = tmp_path / "demo_closed_loop" / "build" / "r1"
    (run / "rollback").mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"status": "rollback_verified"}), encoding="utf-8")
    write_csv(run / "rollback" / "master_output.csv", [(0.1, 1.0), (0.2, 2.1)])
    write_csv(run / "demo_closed_loop_ref.csv", [(0.0, 0.0), (0.1, 1.0), (0.2, 2.0)])
    write_csv(run / "demo_closed_loop_exact.csv", [(0.0, 0.0), (0.1, 1.0), (0.2, 2.0)])
    monkeypatch.setattr(plot_results, "BASE", tmp_path)

    path = plot_results.plot_run("demo_closed_loop", run, plt)

    assert path == run / "demo_closed_loop_comparison.png"
    assert path.stat().st_size > 0


def test_run_without_reference_is_skipped(tmp_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = tmp_path / "single" / "build" / "r1"
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"status": "ran_unvalidated"}), encoding="utf-8")
    write_csv(run / "master_output.csv", [(0.1, 1.0)])

    assert plot_results.plot_run("single", run, plt) is None

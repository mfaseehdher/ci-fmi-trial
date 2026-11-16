"""Plot the Python master against the MATLAB simulation for finished runs.

For every experiment folder with ``build/<run-id>/result.json`` this writes
``build/<run-id>/<experiment>_comparison.png``. Each signal gets one row:
on the left both traces over time, on the right the error over time
(Python master minus MATLAB). When the setup script wrote an exact solution,
the error of each against it is drawn too, so neither is taken as the truth.

The script finds the runs itself, so a new experiment folder needs no change
here. It never changes a run's result: a missing or unreadable trace only
skips that plot.

    py -3.13 docs/experiments/plot_results.py --run-id <run-id> [experiment ...]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent


def read_trace(path: Path) -> dict[str, dict[float, float]]:
    """Columns of a CSV keyed by time rounded to 1e-9 s."""
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = [c for c in (reader.fieldnames or ()) if c != "time"]
        trace: dict[str, dict[float, float]] = {c: {} for c in columns}
        for row in reader:
            t = round(float(row["time"]), 9)
            for column in columns:
                trace[column][t] = float(row[column])
    return trace


def find_inputs(run: Path, experiment: str, result: dict) -> dict | None:
    """Where the master output, MATLAB reference and exact solution are."""
    coupled_output = run / "rollback" / "master_output.csv"
    if coupled_output.is_file():
        reference = run / f"{experiment}_ref.csv"
        exact = run / f"{experiment}_exact.csv"
        return {
            "kind": "coupled, rollback",
            "master": coupled_output,
            "reference": reference if reference.is_file() else None,
            "exact": exact if exact.is_file() else None,
            "columns": None,
        }
    single_output = run / "master_output.csv"
    if single_output.is_file():
        reference = run / result["reference"] if result.get("reference") else None
        comparison = result.get("comparison") or {}
        columns = None
        if comparison.get("output_column") and comparison.get("reference_column"):
            columns = {comparison["output_column"]: comparison["reference_column"]}
        return {
            "kind": "single FMU",
            "master": single_output,
            "reference": reference if reference and reference.is_file() else None,
            "exact": None,
            "columns": columns,
        }
    return None


def paired(a: dict[float, float], b: dict[float, float]) -> tuple[list[float], list[float]]:
    times = sorted(a.keys() & b.keys())
    return times, [a[t] - b[t] for t in times]


def rmse(errors: list[float]) -> float:
    return math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else float("nan")


def plot_run(experiment: str, run: Path, plt) -> Path | None:
    result = json.loads((run / "result.json").read_text(encoding="utf-8"))
    files = find_inputs(run, experiment, result)
    if files is None or files["reference"] is None:
        print(f"[plot] {experiment}: no master output or MATLAB reference, skipped")
        return None
    master = read_trace(files["master"])
    reference = read_trace(files["reference"])
    exact = read_trace(files["exact"]) if files["exact"] else {}
    mapping = files["columns"] or {c: c for c in master if c in reference}
    if not mapping:
        print(f"[plot] {experiment}: master and reference share no signal, skipped")
        return None

    rows = len(mapping)
    figure, axes = plt.subplots(rows, 2, figsize=(13, 3.6 * rows), squeeze=False)
    for row, (out_col, ref_col) in enumerate(mapping.items()):
        values, errors = axes[row]
        ref = reference[ref_col]
        out = master[out_col]
        values.plot(sorted(ref), [ref[t] for t in sorted(ref)],
                    color="black", linewidth=2.2, label="MATLAB simulation")
        values.plot(sorted(out), [out[t] for t in sorted(out)],
                    color="tab:orange", linewidth=1.2, linestyle="--", label="Python master")
        times, diff = paired(out, ref)
        errors.plot(times, diff, color="tab:orange", linewidth=1.2,
                    label=f"master − MATLAB (RMSE {rmse(diff):.2e})")
        if ref_col in exact:
            ex = exact[ref_col]
            values.plot(sorted(ex), [ex[t] for t in sorted(ex)],
                        color="tab:gray", linewidth=1.0, linestyle=":", label="exact solution")
            times, diff = paired(out, ex)
            errors.plot(times, diff, color="tab:blue", linewidth=1.0,
                        label=f"master − exact (RMSE {rmse(diff):.2e})")
            times, diff = paired(ref, ex)
            errors.plot(times, diff, color="tab:green", linewidth=1.0,
                        label=f"MATLAB − exact (RMSE {rmse(diff):.2e})")
        errors.axhline(0.0, color="black", linewidth=0.6)
        values.set_title(f"{out_col}: Python master vs MATLAB")
        errors.set_title(f"{out_col}: error over time")
        for axis in (values, errors):
            axis.set_xlabel("time [s]")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best", fontsize=8)
        values.set_ylabel(out_col)
        errors.set_ylabel("error")

    figure.suptitle(f"{experiment} ({files['kind']}): {result.get('status', '?')}", fontsize=12)
    figure.tight_layout()
    path = run / f"{experiment}_comparison.png"
    figure.savefig(path, dpi=110)
    plt.close(figure)
    print(f"[plot] {experiment}: {path.relative_to(BASE)}")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("experiments", nargs="*", help="Folder names; default is every run found")
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib is not installed; no plots written", file=sys.stderr)
        return 1
    names = args.experiments or sorted(
        p.name for p in BASE.iterdir()
        if (p / "build" / args.run_id / "result.json").is_file()
    )
    written = 0
    for name in names:
        run = BASE / name / "build" / args.run_id
        if not (run / "result.json").is_file():
            print(f"[plot] {name}: no result.json for run {args.run_id}, skipped")
            continue
        try:
            written += plot_run(name, run, plt) is not None
        except Exception as exc:  # a plot must never hide the run's own result
            print(f"[plot] {name}: {type(exc).__name__}: {exc}")
    print(f"[plot] {written} plot(s) written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

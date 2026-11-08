"""Prepare and check one Simulink model per experiment folder.

Only a model name and optional setup script are required in case.json. The
setup may supply a single external input as MATLAB workspace vectors t and u.
Unknown inputs, missing outputs, and missing references are reported, not
filled in with guessed values.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ci_fmi.master_algorithm import run_case
from fmpy import read_model_description
from validation_metrics import calculate_metrics, write_metrics_csv


def matlab_quote(value: Path) -> str:
    return str(value).replace("\\", "/").replace("'", "''")


def prepare(case_dir: Path, run_dir: Path, matlab_exe: str) -> None:
    run_dir.mkdir(parents=True, exist_ok=False)
    command = (
        f"addpath('{matlab_quote(Path(__file__).parent)}'); "
        "mex -setup C; "
        f"prepare_simulink_experiment('{matlab_quote(case_dir)}', "
        f"'{matlab_quote(run_dir)}')"
    )
    result = subprocess.run(
        [matlab_exe, "-batch", command], cwd=ROOT,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        errors="replace", check=False, timeout=900,
    )
    (run_dir / "matlab.log").write_text(result.stdout, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(
            f"MATLAB exited with code {result.returncode}; see matlab.log"
        )
    if not (run_dir / "matlab_status.json").is_file():
        raise RuntimeError("MATLAB produced no status file; see matlab.log")


def signal_columns(path: Path) -> list[str]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        fields = csv.DictReader(stream).fieldnames or []
    return [name for name in fields if name.lower() != "time"]


def matlab_error_summary(message: str) -> str:
    missing = list(dict.fromkeys(re.findall(
        r"Unrecognized function or variable '([^']+)'", message
    )))
    if missing:
        return "Missing MATLAB variables: " + ", ".join(missing)
    field = re.search(r'Unrecognized field name "([^"]+)"', message)
    if field:
        return f"Missing MATLAB simulation output: {field.group(1)}"
    return message.splitlines()[0] if message else "Unknown MATLAB error"


def numeric_rows(path: Path, value_column: str) -> list[tuple[float, float]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if "time" not in (reader.fieldnames or []) or value_column not in (reader.fieldnames or []):
            raise ValueError(f"Missing time or {value_column} in {path.name}")
        rows = [(float(row["time"]), float(row[value_column])) for row in reader]
    if not rows or any(not (math.isfinite(t) and math.isfinite(y)) for t, y in rows):
        raise ValueError(f"Empty or non-finite trace: {path.name}")
    if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"Non-increasing time in {path.name}")
    return rows


def compare_traces(output: Path, reference: Path, output_column: str,
                   reference_column: str) -> dict:
    actual = numeric_rows(output, output_column)
    expected = numeric_rows(reference, reference_column)
    ref_by_time = {round(t, 9): value for t, value in expected}
    paired = []
    for t, value in actual:
        key = round(t, 9)
        if key not in ref_by_time:
            raise ValueError(f"No reference sample at t={t}; no interpolation used")
        paired.append((value, ref_by_time[key]))
    return {
        "samples": len(paired),
        "reference_samples": len(expected),
        **calculate_metrics(paired),
        "reference_peak": max(abs(ref) for _, ref in paired),
        "first_actual": actual[0][1],
        "last_actual": actual[-1][1],
        "first_reference": paired[0][1],
        "last_reference": paired[-1][1],
        "output_column": output_column,
        "reference_column": reference_column,
    }


def evaluate(case_dir: Path, run_dir: Path) -> dict:
    status = json.loads((run_dir / "matlab_status.json").read_text(encoding="utf-8"))
    report: dict = {
        "experiment": case_dir.name,
        "model": status["model"],
        "setup_file": status.get("setup_file", ""),
        "matlab_setup_ok": status["setup_ok"],
        "fmu_export_ok": status["export_ok"],
        "inputs": status["inputs"],
        "outputs": status["outputs"],
        "status": "blocked",
        "reasons": [],
    }
    if not status["setup_ok"]:
        report["reasons"].append("Original MATLAB setup failed: " +
                                 matlab_error_summary(status["setup_error"]))
        report["setup_error"] = status["setup_error"]
    if not status["export_ok"]:
        report["reasons"].append("FMU export failed: " +
                                 matlab_error_summary(status["export_error"]))
        report["export_error"] = status["export_error"]
    if not status["outputs"]:
        report["reasons"].append("No top-level Simulink Outport is exposed")
    if not status["setup_ok"] or not status["export_ok"] or not status["outputs"]:
        return report

    fmu_path = run_dir / status["fmu_file"]
    md = read_model_description(str(fmu_path), validate=False)
    inputs = [v.name for v in md.modelVariables if v.causality == "input"]
    outputs = [v.name for v in md.modelVariables if v.causality == "output"]
    if set(inputs) != set(status["inputs"]) or set(outputs) != set(status["outputs"]):
        report["reasons"].append("FMU ports differ from top-level Simulink ports")
        return report
    step = getattr(md.defaultExperiment, "stepSize", None)
    stop = getattr(md.defaultExperiment, "stopTime", None)
    if not step or not stop:
        report["reasons"].append("FMU does not declare a usable step and stop time")
        return report
    dt, stop = float(step), float(stop)
    if not math.isfinite(dt) or not math.isfinite(stop) or dt <= 0 or stop <= 0:
        report["reasons"].append("FMU declares invalid simulation times")
        return report

    reference_path = run_dir / status["reference_file"] if status["reference_file"] else None
    stimulus_path = run_dir / status["stimulus_file"] if status["stimulus_file"] else None
    if inputs and (stimulus_path is None or not stimulus_path.is_file()):
        report["reasons"].append("Input stimulus is unknown; supply it in the setup")
        return report
    if inputs and set(signal_columns(stimulus_path)) != set(inputs):
        report["reasons"].append("Stimulus columns do not match FMU input ports")
        return report
    if reference_path is None or not reference_path.is_file():
        report["reasons"].append("No original Simulink reference trace to compare")

    components = {"model": {"type": "fmu", "file": fmu_path.name}}
    connections = []
    if inputs:
        components["stimulus"] = {"type": "csv", "file": stimulus_path.name}
        connections.extend({
            "src_comp": "stimulus", "src_port": name,
            "dst_comp": "model", "dst_port": name,
        } for name in inputs)
    output_path = run_dir / "master_output.csv"
    components["logger"] = {"type": "logger", "file": output_path.name}
    connections.extend({
        "src_comp": "model", "src_port": name,
        "dst_comp": "logger", "dst_port": name,
    } for name in outputs)
    cfg = {"start": 0.0, "stop": stop, "dt": dt,
           "components": components, "connections": connections}
    (run_dir / "generated_case.json").write_text(
        json.dumps(cfg, indent=2) + "\n", encoding="utf-8"
    )
    previous_dir = Path.cwd()
    try:
        os.chdir(run_dir)
        run_case(cfg)
    finally:
        os.chdir(previous_dir)
    report["status"] = "ran_unvalidated"
    report["master_output"] = output_path.name
    report["dt"] = dt
    report["stop"] = stop

    if reference_path is not None and reference_path.is_file():
        reference_columns = signal_columns(reference_path)
        if len(outputs) == len(reference_columns) == 1:
            report["comparison"] = compare_traces(
                output_path, reference_path, outputs[0], reference_columns[0]
            )
            report["status"] = "compared"
            report["reference"] = reference_path.name
            write_metrics_csv(
                run_dir / "metrics.csv", case_dir.name,
                {outputs[0]: report["comparison"]},
            )
        else:
            report["reasons"].append("Output-to-reference mapping is ambiguous")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cases", nargs="*", help="Experiment folder names; default is all case.json folders")
    parser.add_argument("--reuse-build", action="store_true", help="Use each folder's existing build directory")
    parser.add_argument("--run-id", help="Output directory name under each case's build directory")
    parser.add_argument("--matlab", default=os.environ.get("MATLAB_EXE", "matlab"))
    args = parser.parse_args()
    run_id = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
        parser.error("--run-id may contain only letters, digits, hyphens, and underscores")
    base = Path(__file__).resolve().parent
    names = args.cases or sorted(p.name for p in base.iterdir()
                                 if p.is_dir() and (p / "case.json").is_file())
    reports = []
    for name in names:
        case_dir = (base / name).resolve()
        if case_dir.parent != base or not (case_dir / "case.json").is_file():
            parser.error(f"Unknown experiment folder: {name}")
        run_dir = case_dir / "build" if args.reuse_build else case_dir / "build" / run_id
        print(f"\n=== {name} ===", flush=True)
        try:
            if not args.reuse_build:
                prepare(case_dir, run_dir, args.matlab)
            report = evaluate(case_dir, run_dir)
        except Exception as err:
            report = {"experiment": name, "status": "error", "error": str(err),
                      "traceback": traceback.format_exc()}
        reports.append(report)
        (run_dir / "result.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps({key: value for key, value in report.items()
                          if key not in ("traceback", "setup_error", "export_error")},
                         indent=2), flush=True)
    print("\nSUMMARY", json.dumps({r["experiment"]: r["status"] for r in reports}))
    return 0 if all(r["status"] == "compared" for r in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

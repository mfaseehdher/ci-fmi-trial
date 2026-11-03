"""Build and compare the controller/plant experiments from the earlier Ci-FMI repo.

The MATLAB setup scripts export two FMUs and a monolithic reference. Their
feedback edges are explicitly delayed. This runner removes the scripts' manual
step_order so the current master must determine its own execution order.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
BASE = Path(__file__).resolve().parent

CASES = (
    "motor_closed_loop",
    "rc_circuit_closed_loop",
    "thermal_tank_closed_loop",
    "pendulum_closed_loop",
    "inverted_pendulum_lqr_closed_loop",
    "mass_spring_damper_closed_loop",
    "stiff_rlc_circuit_closed_loop",
    "stiff_two_time_scale_closed_loop",
)


def selected_cases(names: list[str]) -> list[str]:
    unknown = set(names) - set(CASES)
    if unknown:
        raise ValueError("Unknown coupled case: " + ", ".join(sorted(unknown)))
    return names or list(CASES)


def run_dir(name: str, run_id: str) -> Path:
    return BASE / name / "build" / run_id


def matlab_quote(path: Path) -> str:
    return path.as_posix().replace("'", "''")


def build_one(name: str, run_id: str, matlab: str) -> dict:
    source = BASE / name
    output = run_dir(name, run_id)
    output.mkdir(parents=True, exist_ok=False)
    for path in source.iterdir():
        if path.suffix.lower() in {".m", ".slx", ".mdl"}:
            shutil.copy2(path, output / path.name)
    script = f"setup_{name}"
    command = f"cd('{matlab_quote(output)}'); mex -setup C; {script}"
    try:
        process = subprocess.run(
            [matlab, "-batch", command], cwd=ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, errors="replace",
            timeout=1800, check=False,
        )
        (output / "matlab.log").write_text(process.stdout, encoding="utf-8")
        expected = [output / f"{name}.json", output / f"{name}_ref.csv"]
        fmus = list(output.glob("*.fmu"))
        status = "built" if process.returncode == 0 and all(p.is_file() for p in expected) and len(fmus) >= 2 else "error"
        report = {
            "experiment": name, "status": status, "matlab_exit_code": process.returncode,
            "fmus": sorted(p.name for p in fmus),
            "expected_files_present": all(p.is_file() for p in expected),
        }
    except Exception as exc:
        report = {"experiment": name, "status": "error", "error": str(exc)}
    (output / "build_result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def numeric_trace(path: Path, column: str) -> list[tuple[float, float]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if "time" not in (reader.fieldnames or ()) or column not in (reader.fieldnames or ()):
            raise ValueError(f"Missing time or {column} in {path}")
        rows = [(float(row["time"]), float(row[column])) for row in reader]
    if not rows or any(not (math.isfinite(t) and math.isfinite(y)) for t, y in rows):
        raise ValueError(f"Empty or non-finite trace in {path}")
    if any(later[0] <= earlier[0] for earlier, later in zip(rows, rows[1:])):
        raise ValueError(f"Non-increasing time in {path}")
    return rows


def compare(output: Path, reference: Path) -> dict:
    with reference.open(newline="", encoding="utf-8-sig") as stream:
        columns = [c for c in (csv.DictReader(stream).fieldnames or ()) if c != "time"]
    if not columns:
        raise ValueError(f"No reference signal columns in {reference}")
    metrics = {}
    for column in columns:
        actual = numeric_trace(output, column)
        expected = dict((round(t, 9), value) for t, value in numeric_trace(reference, column))
        errors = []
        for t, value in actual:
            key = round(t, 9)
            if key not in expected:
                raise ValueError(f"No reference sample for {column} at t={t}")
            errors.append(value - expected[key])
        metrics[column] = {
            "samples": len(errors),
            "rmse": math.sqrt(sum(e * e for e in errors) / len(errors)),
            "max_abs_error": max(abs(e) for e in errors),
            "first_actual": actual[0][1],
            "first_reference": expected[round(actual[0][0], 9)],
            "last_actual": actual[-1][1],
            "last_reference": expected[round(actual[-1][0], 9)],
        }
    return metrics


def check_one(name: str, run_id: str) -> dict:
    sys.path.insert(0, str(ROOT / "src"))
    from ci_fmi.master_algorithm import run_case

    output = run_dir(name, run_id)
    build_report = json.loads((output / "build_result.json").read_text(encoding="utf-8"))
    if build_report["status"] != "built":
        return {"experiment": name, "status": "blocked", "reason": "MATLAB build did not complete"}
    cfg = json.loads((output / f"{name}.json").read_text(encoding="utf-8"))
    cfg.pop("step_order", None)
    for component in cfg["components"].values():
        if component["type"] == "logger":
            component["file"] = "master_output.csv"
    (output / "generated_case.json").write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    previous_dir = Path.cwd()
    try:
        os.chdir(output)
        run_case(cfg)
    finally:
        os.chdir(previous_dir)
    return {
        "experiment": name,
        "status": "compared",
        "feedback": "explicitly delayed in original configuration",
        "manual_step_order_removed": True,
        "metrics": compare(output / "master_output.csv", output / f"{name}_ref.csv"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("build", "check"))
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--matlab", default=os.environ.get("MATLAB_EXE", "matlab"))
    parser.add_argument("cases", nargs="*")
    args = parser.parse_args()
    if not args.run_id.replace("-", "").replace("_", "").isalnum():
        parser.error("run-id must contain only letters, digits, hyphens, or underscores")
    names = selected_cases(args.cases)
    reports = []
    for name in names:
        print(f"\n=== {name}: {args.phase} ===", flush=True)
        try:
            report = build_one(name, args.run_id, args.matlab) if args.phase == "build" else check_one(name, args.run_id)
        except Exception as exc:
            report = {"experiment": name, "status": "error", "error": str(exc),
                      "traceback": traceback.format_exc()}
        if args.phase == "check":
            output = run_dir(name, args.run_id)
            output.mkdir(parents=True, exist_ok=True)
            (output / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        reports.append(report)
        print(json.dumps({k: v for k, v in report.items() if k != "traceback"}, indent=2), flush=True)
    good = "built" if args.phase == "build" else "compared"
    return 0 if all(report["status"] == good for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

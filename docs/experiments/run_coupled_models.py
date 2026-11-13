"""Build the controller/plant experiments and use them as rollback tests.

The MATLAB setup scripts export a controller FMU, a plant FMU and a monolithic
closed-loop reference. The plant-to-controller feedback is a same-time
connection: neither the configuration nor the Simulink reference delays it.
The master must therefore treat each pair as a loop and solve every
communication step by FMU state rollback.

For each case the check phase runs, each in its own Python process because
MATLAB FMUs can be instantiated only once per process:

* a restore probe for every FMU: save state, step, restore, repeat the step;
* ``rollback``: the configuration as exported; its output is compared with
  the MATLAB simulation of the same closed loop;
* ``no_restore``: the same run with ``setFMUstate`` disabled. It is not a
  result to compare; it only shows that the check notices when the FMU state
  is not restored.

The runner removes the scripts' manual step_order so the current master must
determine its own execution plan.
"""

from __future__ import annotations

import argparse
import copy
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
sys.path.insert(0, str(BASE))

from validation_metrics import calculate_metrics, write_metrics_csv

CASES = (
    "motor_closed_loop",
    "rc_circuit_closed_loop",
    "thermal_tank_closed_loop",
    "pendulum_closed_loop",
    "inverted_pendulum_lqr_closed_loop",
    "mass_spring_damper_closed_loop",
    "stiff_rlc_circuit_closed_loop",
    "stiff_two_time_scale_closed_loop",
    "cruise_control_closed_loop",
)

VARIANTS = ("rollback", "no_restore")
PROBE_WARMUP_STEPS = 10
# A disabled restore must change the trace by more than this fraction of the
# signal range. It is far above the loop tolerances (1e-9 absolute, 1e-7
# relative), so solver noise cannot count as a detected difference.
SENSITIVITY_FRACTION = 1e-6
WORKER_TIMEOUT_S = 1800


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


# ──────────────────────────────────────────────────────────────
# Traces and metrics
# ──────────────────────────────────────────────────────────────

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


def signal_columns(path: Path) -> list[str]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        columns = [c for c in (csv.DictReader(stream).fieldnames or ()) if c != "time"]
    if not columns:
        raise ValueError(f"No signal columns in {path}")
    return columns


def compare(output: Path, reference: Path) -> dict:
    metrics = {}
    for column in signal_columns(reference):
        actual = numeric_trace(output, column)
        expected = dict((round(t, 9), value) for t, value in numeric_trace(reference, column))
        paired = []
        for t, value in actual:
            key = round(t, 9)
            if key not in expected:
                raise ValueError(f"No reference sample for {column} at t={t}")
            paired.append((value, expected[key]))
        metrics[column] = {
            "samples": len(paired),
            **calculate_metrics(paired),
            "first_actual": actual[0][1],
            "first_reference": expected[round(actual[0][0], 9)],
            "last_actual": actual[-1][1],
            "last_reference": expected[round(actual[-1][0], 9)],
        }
    return metrics


def trace_difference(first: Path, second: Path, columns: list[str]) -> dict:
    """Largest difference between two master traces at shared timestamps."""
    largest = 0.0
    scale = 0.0
    for column in columns:
        a = dict((round(t, 9), y) for t, y in numeric_trace(first, column))
        b = dict((round(t, 9), y) for t, y in numeric_trace(second, column))
        shared = sorted(a.keys() & b.keys())
        if not shared:
            raise ValueError(f"Traces share no timestamps for {column}")
        largest = max(largest, max(abs(a[t] - b[t]) for t in shared))
        scale = max(scale, max(a.values()) - min(a.values()))
    return {"max_abs_difference": largest, "signal_range": scale}


# ──────────────────────────────────────────────────────────────
# Configuration helpers
# ──────────────────────────────────────────────────────────────

def feedback_connections(cfg: dict) -> list[dict]:
    return [c for c in cfg["connections"]
            if c["src_comp"] == "plant" and c["dst_comp"] == "controller"]


def prepare_config(cfg: dict) -> dict:
    """Validate an exported configuration and drop its manual order."""
    cfg = copy.deepcopy(cfg)
    cfg.pop("step_order", None)
    feedback = feedback_connections(cfg)
    if not feedback:
        raise ValueError("Configuration has no plant-to-controller feedback")
    delayed = [f"{c['src_comp']}.{c['src_port']}->{c['dst_comp']}.{c['dst_port']}"
               for c in cfg["connections"] if c.get("delayed")]
    if delayed:
        raise ValueError(
            "Configuration still delays " + ", ".join(delayed)
            + "; rebuild it with the current setup script"
        )
    return cfg


def fmu_components(cfg: dict) -> dict[str, str]:
    return {name: spec["file"] for name, spec in cfg["components"].items()
            if spec["type"] == "fmu"}


# ──────────────────────────────────────────────────────────────
# Rollback evidence
# ──────────────────────────────────────────────────────────────

def summarize_loop_trace(trace: list[dict]) -> dict:
    """Iterations per communication step from solve_rollback_loop's trace."""
    steps: dict[float, dict] = {}
    for entry in trace:
        step = steps.setdefault(entry["time_start"], {"iterations": 0, "accepted": False})
        step["iterations"] = max(step["iterations"], entry["iteration"])
        step["accepted"] = step["accepted"] or bool(entry["accepted"])
        step["final_residual"] = entry["max_residual"]
    iterations = [step["iterations"] for step in steps.values()]
    return {
        "communication_steps": len(steps),
        "accepted_steps": sum(step["accepted"] for step in steps.values()),
        "steps_with_retries": sum(count > 1 for count in iterations),
        "trial_steps": len(trace),
        "max_iterations": max(iterations, default=0),
        "mean_iterations": sum(iterations) / len(iterations) if iterations else 0.0,
    }


def rollback_stage(plan: list[dict] | None) -> dict | None:
    for stage in plan or ():
        if stage.get("kind") == "algebraic_loop" and stage.get("evaluation") == "rollback_step":
            return stage
    return None


def restore_sensitivity(no_restore: dict, difference: dict | None) -> dict:
    """Did disabling setFMUstate change the outcome?"""
    if no_restore.get("status") != "completed":
        return {"detected": True, "evidence": "no_restore run failed: " + str(no_restore.get("error"))}
    if difference is None:
        return {"detected": False, "evidence": "no trace difference could be computed"}
    limit = SENSITIVITY_FRACTION * max(difference["signal_range"], 1.0)
    return {
        "detected": difference["max_abs_difference"] > limit,
        "evidence": (f"max |no_restore - rollback| = {difference['max_abs_difference']:.3e}"
                     f" (limit {limit:.1e})"),
    }


def rollback_reasons(expected_steps: int, probes: dict, rollback: dict,
                     sensitivity: dict) -> list[str]:
    """Every reason the rollback check is not satisfied; empty means verified."""
    reasons = []
    for name, probe in probes.items():
        if probe.get("status") != "completed":
            reasons.append(f"restore probe for {name} failed: {probe.get('error')}")
        elif not probe["replay_identical"]:
            reasons.append(f"{name}: repeating a step after restore gave different outputs")
    if probes and not any(p.get("stateful") for p in probes.values()):
        reasons.append("no FMU has observable state, so a missing restore cannot be seen")

    stage = rollback_stage(rollback.get("execution_plan"))
    if stage is None:
        reasons.append("the master did not plan the feedback loop as a rollback loop")
    elif set(stage["components"]) != {"controller", "plant"}:
        reasons.append(f"rollback loop contains {stage['components']}, expected controller and plant")

    if rollback.get("status") != "completed":
        reasons.append("rollback run failed: " + str(rollback.get("error")))
    else:
        loop = rollback["loop"]
        if loop["accepted_steps"] != expected_steps:
            reasons.append(f"{loop['accepted_steps']} of {expected_steps} steps were accepted by rollback")
        if loop["steps_with_retries"] == 0:
            reasons.append("no step needed a retry, so restore was never exercised")

    if not sensitivity["detected"]:
        reasons.append("disabling setFMUstate did not change the result: " + sensitivity["evidence"])
    return reasons


# ──────────────────────────────────────────────────────────────
# Workers: each runs in its own process
# ──────────────────────────────────────────────────────────────

def _master():
    sys.path.insert(0, str(ROOT / "src"))
    import ci_fmi.master_algorithm as master
    return master


def probe_inputs(names: list[str]) -> dict[str, float]:
    return {name: 1.0 / (index + 1) for index, name in enumerate(names)}


def probe_worker(fmu_file: str, dt: float, output: Path) -> None:
    """Save, step, restore and repeat the step on one FMU."""
    master = _master()
    report: dict = {"fmu": fmu_file, "warmup_steps": PROBE_WARMUP_STEPS}
    component = master.FMUComponent("probe", fmu_file)
    try:
        component.instantiate(0.0)
        component.configure_stepping(dt)
        report["supports_rollback"] = component.supports_rollback
        # Distinct values, so an error signal such as reference - measured
        # is not zero and the FMU's state actually moves.
        report["inputs"] = probe_inputs(component.inputs_list)
        for name, value in report["inputs"].items():
            component.set_input(name, value)
        t = 0.0
        for _ in range(PROBE_WARMUP_STEPS):
            component.do_step(t, dt)
            t = round(t + dt, 12)
        snapshot = component.save_state()
        component.trial_step(t, dt)
        first = dict(component.outputs)
        component.restore_state(snapshot)
        component.trial_step(t, dt)
        replay = dict(component.outputs)
        # Without restoring, the next step starts from the advanced state.
        component.trial_step(round(t + dt, 12), dt)
        continued = dict(component.outputs)
        component.free_state(snapshot)
        report.update({
            "status": "completed",
            "probe_time": t,
            "first": first, "replay": replay, "next_step_without_restore": continued,
            "replay_identical": all(first[k] == replay[k] for k in first),
            "stateful": any(abs(continued[k] - first[k]) > 1e-12 for k in first),
        })
    except Exception as exc:
        report.update({"status": "error", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        try:
            component.terminate()
        except Exception:
            pass
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def variant_worker(case_file: Path, mode: str, output: Path) -> None:
    """Run one variant of the coupled case and record how the loop was solved."""
    master = _master()
    cfg = json.loads(case_file.read_text(encoding="utf-8"))
    output.mkdir(parents=True, exist_ok=True)
    for component in cfg["components"].values():
        if component["type"] == "logger":
            component["file"] = str(output / "master_output.csv")

    if mode == "no_restore":
        def restore_bookkeeping_only(self, snapshot):
            # Deliberately skip fmu.setFMUstate; keep the master's own records.
            self.current_time = snapshot[1]
            self.inputs = dict(snapshot[2])
            self.outputs = dict(snapshot[3])
        master.FMUComponent.restore_state = restore_bookkeeping_only

    plans: list = []
    planner = master.build_execution_plan

    def recording_planner(*args, **kwargs):
        plan = planner(*args, **kwargs)
        plans.append(plan)
        return plan

    master.build_execution_plan = recording_planner
    trace: list[dict] = []
    report: dict = {"mode": mode}
    try:
        master.run_case(cfg, loop_trace=trace)
        report["status"] = "completed"
    except Exception as exc:
        report.update({"status": "error", "error": f"{type(exc).__name__}: {exc}"})
    report["execution_plan"] = plans[0] if plans else None
    report["loop"] = summarize_loop_trace(trace)
    if trace:
        with (output / "loop_iterations.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_start", "iteration", "max_residual", "accepted"])
            for entry in trace:
                writer.writerow([entry["time_start"], entry["iteration"],
                                 entry["max_residual"], entry["accepted"]])
    (output / "variant.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def run_worker(arguments: list[str], cwd: Path, log: Path) -> int:
    with log.open("w", encoding="utf-8") as stream:
        process = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "worker", *arguments],
            cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, text=True,
            timeout=WORKER_TIMEOUT_S, check=False,
        )
    return process.returncode


def read_report(path: Path, fallback_error: str) -> dict:
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"status": "error", "error": fallback_error}


# ──────────────────────────────────────────────────────────────
# Check phase
# ──────────────────────────────────────────────────────────────

def check_one(name: str, run_id: str) -> dict:
    output = run_dir(name, run_id)
    build_report = json.loads((output / "build_result.json").read_text(encoding="utf-8"))
    if build_report["status"] != "built":
        return {"experiment": name, "status": "blocked", "reason": "MATLAB build did not complete"}
    cfg = prepare_config(json.loads((output / f"{name}.json").read_text(encoding="utf-8")))
    case_file = output / "generated_case.json"
    case_file.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    reference = output / f"{name}_ref.csv"
    dt = float(cfg["dt"])
    expected_steps = round((float(cfg["stop"]) - float(cfg.get("start", 0.0))) / dt)

    probes = {}
    for component, fmu_file in fmu_components(cfg).items():
        probe_file = output / f"restore_probe_{component}.json"
        code = run_worker(["probe", "--fmu", fmu_file, "--dt", repr(dt), "--out", str(probe_file)],
                          output, output / f"restore_probe_{component}.log")
        probes[component] = read_report(probe_file, f"probe process exited with {code}")

    variants = {}
    for mode in VARIANTS:
        directory = output / mode
        code = run_worker(["variant", "--case", str(case_file), "--mode", mode, "--out", str(directory)],
                          output, output / f"{mode}.log")
        variants[mode] = read_report(directory / "variant.json", f"{mode} process exited with {code}")

    metrics = None
    if variants["rollback"].get("status") == "completed":
        metrics = compare(output / "rollback" / "master_output.csv", reference)
        write_metrics_csv(output / "metrics.csv", name, metrics)

    # Optional closed-form solution written by the setup script. Both the
    # master and the MATLAB simulation are measured against it, so neither is
    # treated as the truth.
    exact = output / f"{name}_exact.csv"
    exact_solution = None
    if metrics is not None and exact.is_file():
        exact_solution = {
            "python_master_rollback": compare(output / "rollback" / "master_output.csv", exact),
            "matlab_simulation": compare(reference, exact),
        }
        write_metrics_csv(output / "metrics_exact_master.csv", name,
                          exact_solution["python_master_rollback"])
        write_metrics_csv(output / "metrics_exact_matlab.csv", name,
                          exact_solution["matlab_simulation"])

    difference = None
    if variants["rollback"].get("status") == "completed" and variants["no_restore"].get("status") == "completed":
        difference = trace_difference(output / "rollback" / "master_output.csv",
                                      output / "no_restore" / "master_output.csv",
                                      signal_columns(reference))
    sensitivity = restore_sensitivity(variants["no_restore"], difference)
    reasons = rollback_reasons(expected_steps, probes, variants["rollback"], sensitivity)

    return {
        "experiment": name,
        "status": "rollback_verified" if not reasons else "rollback_not_verified",
        "reasons": reasons,
        "feedback": "same-time plant-to-controller connection, no delay",
        "manual_step_order_removed": True,
        "communication_steps": expected_steps,
        "restore_probes": {k: {key: v.get(key) for key in
                               ("status", "error", "replay_identical", "stateful")}
                           for k, v in probes.items()},
        "rollback": {key: variants["rollback"].get(key) for key in ("status", "error", "loop")},
        "rollback_stage": rollback_stage(variants["rollback"].get("execution_plan")),
        "no_restore": {"status": variants["no_restore"].get("status"),
                       "error": variants["no_restore"].get("error"),
                       "difference_from_rollback": difference},
        "restore_sensitivity": sensitivity,
        "comparison": "Python master with rollback vs MATLAB closed-loop simulation",
        "metrics": metrics,
        "exact_solution": exact_solution,
    }


def worker_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="run_coupled_models.py worker")
    parser.add_argument("kind", choices=("probe", "variant"))
    parser.add_argument("--fmu")
    parser.add_argument("--dt", type=float)
    parser.add_argument("--case", type=Path)
    parser.add_argument("--mode", choices=VARIANTS)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.kind == "probe":
        probe_worker(args.fmu, args.dt, args.out)
    else:
        variant_worker(args.case, args.mode, args.out)
    return 0


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "worker":
        return worker_main(sys.argv[2:])
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
    good = "built" if args.phase == "build" else "rollback_verified"
    return 0 if all(report["status"] == good for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

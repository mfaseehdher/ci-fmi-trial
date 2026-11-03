"""Run one or all MATLAB FMU experiments and check the observed outcomes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path


EXPERIMENT = Path(__file__).resolve().parent
RESULTS = EXPERIMENT / "results"
MARKER = "CASE_RESULT:"


def check_case(path: Path) -> dict[str, object]:
    config = json.loads(path.read_text(encoding="utf-8"))
    expected = config["expect"]
    process = subprocess.run(
        [sys.executable, str(EXPERIMENT / "run_one_case.py"), str(path)],
        cwd=EXPERIMENT,
        capture_output=True,
        text=True,
        check=False,
    )
    markers = [
        line.removeprefix(MARKER)
        for line in process.stdout.splitlines()
        if line.startswith(MARKER)
    ]
    result = json.loads(markers[0]) if len(markers) == 1 else None
    case_name = path.parent.name
    report: dict[str, object] = {"case": case_name, "status": "FAIL"}

    if result is None:
        report["reason"] = "Worker did not return a structured result"
    elif expected["result"] == "rejected":
        error = result.get("error")
        reason = result.get("reason", "")
        if (process.returncode != 0 and error == expected["error"]
                and expected["reason_contains"] in reason):
            report.update(status="EXPECTED_REJECTION", reason=reason)
        else:
            report["reason"] = (
                f"Expected {expected['error']} containing "
                f"{expected['reason_contains']!r}; got {error}: {reason}"
            )
    elif expected["result"] == "pass":
        if process.returncode != 0 or result.get("error"):
            report["reason"] = f"Unexpected {result.get('error')}: {result.get('reason')}"
        else:
            output = EXPERIMENT / config["components"]["log"]["file"]
            if not output.is_file():
                report["reason"] = f"Missing result CSV: {output}"
            else:
                with output.open(newline="", encoding="utf-8") as handle:
                    rows = list(csv.DictReader(handle))
                columns = expected.get("outputs", {"sum": expected.get("values", [])})
                tolerance = float(expected.get("tolerance", 1e-8))
                reference_kind = expected.get("reference", "matlab")
                reference = EXPERIMENT / "references" / f"{case_name}.csv"
                if reference_kind == "matlab" and not reference.is_file():
                    report["reason"] = f"Missing MATLAB reference: {reference}"
                elif reference_kind not in ("matlab", "analytical"):
                    report["reason"] = f"Unknown reference type {reference_kind!r}"
                else:
                    if reference_kind == "matlab":
                        with reference.open(newline="", encoding="utf-8") as handle:
                            reference_rows = list(csv.DictReader(handle))
                    else:
                        reference_rows = None
                    matches = bool(rows) and all(
                        len(rows) == len(targets) for targets in columns.values()
                    )
                    if reference_rows is not None:
                        matches = matches and len(rows) == len(reference_rows)
                    for index, row in enumerate(rows):
                        expected_time = float(config.get("start", 0.0)) + (index + 1) * float(config["dt"])
                        matches = matches and math.isclose(
                            float(row["time"]), expected_time, rel_tol=0, abs_tol=1e-9
                        )
                        for column, targets in columns.items():
                            if column not in row or index >= len(targets):
                                matches = False
                                continue
                            actual = float(row[column])
                            matches = matches and math.isfinite(actual) and math.isclose(
                                actual, float(targets[index]), rel_tol=0, abs_tol=tolerance
                            )
                            if reference_rows is not None and index < len(reference_rows):
                                ref = reference_rows[index]
                                matches = matches and math.isclose(
                                    float(row["time"]), float(ref["time"]),
                                    rel_tol=0, abs_tol=1e-9,
                                ) and column in ref and math.isclose(
                                    actual, float(ref[column]), rel_tol=0, abs_tol=tolerance
                                )
                    if matches:
                        report.update(status="PASS", values={
                            column: [float(row[column]) for row in rows]
                            for column in columns
                        })
                    else:
                        report["reason"] = (
                            f"Master rows {rows}, reference {reference_rows}, "
                            f"expected {columns} (tolerance {tolerance})"
                        )
    else:
        report["reason"] = f"Unknown expectation {expected['result']!r}"

    report["stdout"] = "\n".join(
        line for line in process.stdout.splitlines()
        if not line.startswith(MARKER)
    )
    if process.stderr.strip():
        report["stderr"] = process.stderr.strip()
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", nargs="?", help="Experiment folder name; omit to run all")
    args = parser.parse_args()

    paths = sorted(EXPERIMENT.glob("*/case.json"))
    if args.case:
        paths = [path for path in paths if path.parent.name == args.case]
    if not paths:
        raise ValueError(f"No experiment configuration found for {args.case!r}")

    RESULTS.mkdir(exist_ok=True)
    reports = [check_case(path) for path in paths]

    (RESULTS / "summary.json").write_text(
        json.dumps(reports, indent=2) + "\n", encoding="utf-8"
    )
    table = ["| Case | Status | Detail |", "|---|---|---|"]
    for report in reports:
        detail = str(report.get("reason", ""))[:180].replace("|", "\\|").replace("\n", " ")
        table.append(f"| {report['case']} | {report['status']} | {detail} |")
        print(f"{report['status']}: {report['case']}")
    (RESULTS / "summary.md").write_text("\n".join(table) + "\n", encoding="utf-8")
    return int(any(report["status"] == "FAIL" for report in reports))


if __name__ == "__main__":
    sys.exit(main())

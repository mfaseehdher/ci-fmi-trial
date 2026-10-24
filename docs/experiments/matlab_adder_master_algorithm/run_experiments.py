"""Run each MATLAB FMU case separately and check its declared outcome."""

from __future__ import annotations

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
    report: dict[str, object] = {"case": path.stem, "status": "FAIL"}

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
                actual = [float(row["sum"]) for row in rows]
                values = expected["values"]
                reference = EXPERIMENT / "references" / f"{path.stem}.csv"
                if not reference.is_file():
                    report["reason"] = f"Missing MATLAB reference: {reference}"
                else:
                    with reference.open(newline="", encoding="utf-8") as handle:
                        reference_rows = list(csv.DictReader(handle))
                    matches = (
                        len(rows) == len(values) == len(reference_rows)
                        and all(
                            math.isclose(float(row["time"]), float(ref["time"]), rel_tol=0, abs_tol=1e-9)
                            and math.isclose(float(row["sum"]), float(ref["sum"]), rel_tol=0, abs_tol=1e-8)
                            and math.isclose(float(row["sum"]), target, rel_tol=0, abs_tol=1e-8)
                            for row, ref, target in zip(rows, reference_rows, values)
                        )
                    )
                    if matches:
                        report.update(status="PASS", values=actual)
                    else:
                        report["reason"] = f"Master {actual}, MATLAB reference {[row['sum'] for row in reference_rows]}, expected {values}"
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
    RESULTS.mkdir(exist_ok=True)
    reports = [check_case(path) for path in sorted((EXPERIMENT / "configs").glob("*.json"))]
    if not reports:
        raise RuntimeError("No experiment configurations found")

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

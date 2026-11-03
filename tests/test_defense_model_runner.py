import csv
import json
from pathlib import Path

import pytest

from docs.experiments.run_defense_models import compare_traces, evaluate, matlab_error_summary


def write_trace(path: Path, rows: list[tuple[float, float]], name: str) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time", name])
        writer.writerows(rows)


def test_compare_pairs_only_equal_times(tmp_path: Path) -> None:
    output = tmp_path / "output.csv"
    reference = tmp_path / "reference.csv"
    write_trace(output, [(0.1, 2.0), (0.2, 4.0)], "Out1")
    write_trace(reference, [(0.0, 0.0), (0.1, 2.1), (0.2, 3.8)], "Speed")

    result = compare_traces(output, reference, "Out1", "Speed")

    assert result["samples"] == 2
    assert result["reference_samples"] == 3
    assert result["max_abs_error"] == pytest.approx(0.2)


def test_compare_refuses_missing_timestamp(tmp_path: Path) -> None:
    output = tmp_path / "output.csv"
    reference = tmp_path / "reference.csv"
    write_trace(output, [(0.1, 2.0)], "Out1")
    write_trace(reference, [(0.2, 2.0)], "Speed")

    with pytest.raises(ValueError, match="No reference sample"):
        compare_traces(output, reference, "Out1", "Speed")


def test_no_output_is_blocked_even_when_export_succeeded(tmp_path: Path) -> None:
    case_dir = tmp_path / "case"
    run_dir = case_dir / "build"
    run_dir.mkdir(parents=True)
    status = {
        "model": "unobservable", "setup_ok": False,
        "setup_error": "Missing yout", "export_ok": True,
        "export_error": "", "inputs": [], "outputs": [],
    }
    (run_dir / "matlab_status.json").write_text(json.dumps(status), encoding="utf-8")

    result = evaluate(case_dir, run_dir)

    assert result["status"] == "blocked"
    assert any("Outport" in reason for reason in result["reasons"])


def test_matlab_error_summary_names_missing_variables() -> None:
    error = ("Unrecognized function or variable 'Ki'.\n"
             "Unrecognized function or variable 'Kp'.\n"
             "Unrecognized function or variable 'Ki'.")

    assert matlab_error_summary(error) == "Missing MATLAB variables: Ki, Kp"

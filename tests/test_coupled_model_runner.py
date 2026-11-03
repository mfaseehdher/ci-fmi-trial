import csv

import pytest

from docs.experiments.run_coupled_models import compare, selected_cases


def write_trace(path, columns, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def test_compare_all_matching_signals_at_equal_times(tmp_path):
    output = tmp_path / "output.csv"
    reference = tmp_path / "reference.csv"
    write_trace(output, ["time", "Position", "Angle"],
                [(0.1, 1.0, 2.0), (0.2, 3.0, 4.0)])
    write_trace(reference, ["time", "Position", "Angle"],
                [(0.0, 0.0, 0.0), (0.1, 1.1, 2.2), (0.2, 3.2, 4.4)])

    metrics = compare(output, reference)

    assert set(metrics) == {"Position", "Angle"}
    assert metrics["Position"]["samples"] == 2
    assert metrics["Position"]["max_abs_error"] == pytest.approx(0.2)
    assert metrics["Angle"]["max_abs_error"] == pytest.approx(0.4)


def test_compare_rejects_unmatched_time(tmp_path):
    output = tmp_path / "output.csv"
    reference = tmp_path / "reference.csv"
    write_trace(output, ["time", "Voltage"], [(0.1, 1.0)])
    write_trace(reference, ["time", "Voltage"], [(0.2, 1.0)])

    with pytest.raises(ValueError, match="No reference sample"):
        compare(output, reference)


def test_case_selection_rejects_unknown_case():
    with pytest.raises(ValueError, match="Unknown coupled case"):
        selected_cases(["not_a_case"])

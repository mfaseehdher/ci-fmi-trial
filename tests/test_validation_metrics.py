import csv
import math

import pytest

from docs.experiments.validation_metrics import (
    calculate_metrics,
    windowed_dtw_mae,
    write_metrics_csv,
)


def test_error_metrics_use_matching_samples():
    metrics = calculate_metrics([(0.0, 0.0), (2.0, 1.0), (2.0, 2.0)])

    assert metrics["rmse"] == pytest.approx(math.sqrt(1 / 3))
    assert metrics["mae"] == pytest.approx(1 / 3)
    assert metrics["max_abs_error"] == 1.0
    assert metrics["reference_range"] == 2.0
    assert metrics["relative_rmse_percent"] == pytest.approx(50 * math.sqrt(1 / 3))


def test_constant_reference_has_no_relative_rmse():
    metrics = calculate_metrics([(1.0, 2.0), (3.0, 2.0)])

    assert metrics["relative_rmse_percent"] is None
    assert metrics["dtw_mae_window_5"] >= 0


def test_windowed_dtw_reports_shape_without_hiding_pointwise_error():
    reference = [0.0, 1.0, 2.0, 3.0]
    actual = [0.0, 0.0, 1.0, 2.0]

    assert windowed_dtw_mae(reference, reference) == 0.0
    assert windowed_dtw_mae(actual, reference) < sum(
        abs(a - b) for a, b in zip(actual, reference)
    ) / len(actual)


def test_metrics_csv_contains_all_numbers(tmp_path):
    path = tmp_path / "metrics.csv"
    metrics = calculate_metrics([(0.0, 0.0), (2.0, 1.0), (2.0, 2.0)])

    write_metrics_csv(path, "example", {"Voltage": {"samples": 3, **metrics}})

    with path.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))
    assert row["experiment"] == "example"
    assert row["signal"] == "Voltage"
    assert float(row["mae"]) == pytest.approx(1 / 3)
    assert float(row["relative_rmse_percent"]) == pytest.approx(50 * math.sqrt(1 / 3))

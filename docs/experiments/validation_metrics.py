"""Error measurements for master output and MATLAB reference at matching times."""

import csv
import math
from pathlib import Path


DTW_WINDOW = 5


def windowed_dtw_mae(actual: list[float], reference: list[float], window: int = DTW_WINDOW) -> float:
    """Mean absolute DTW cost, allowing at most `window` samples of time shift."""
    if not actual or not reference or window < 0:
        raise ValueError("DTW needs nonempty signals and a nonnegative window")
    window = max(window, abs(len(actual) - len(reference)))
    previous = {0: (0.0, 0)}
    for i, value in enumerate(actual, 1):
        current = {}
        for j in range(max(1, i - window), min(len(reference), i + window) + 1):
            options = [cell for cell in (
                previous.get(j), current.get(j - 1), previous.get(j - 1)
            ) if cell is not None]
            if options:
                cost, length = min(options, key=lambda cell: (cell[0], cell[1]))
                current[j] = (cost + abs(value - reference[j - 1]), length + 1)
        previous = current
    cost, length = previous[len(reference)]
    return cost / length


def calculate_metrics(paired: list[tuple[float, float]]) -> dict:
    """Each pair is (master value, MATLAB value) at the same timestamp."""
    if not paired:
        raise ValueError("No matching samples to compare")
    actual = [value for value, _ in paired]
    reference = [value for _, value in paired]
    errors = [value - target for value, target in paired]
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
    reference_range = max(reference) - min(reference)
    return {
        "rmse": rmse,
        "mae": sum(abs(error) for error in errors) / len(errors),
        "max_abs_error": max(abs(error) for error in errors),
        "reference_range": reference_range,
        "relative_rmse_percent": 100 * rmse / reference_range if reference_range else None,
        "dtw_mae_window_5": windowed_dtw_mae(actual, reference),
    }


def write_metrics_csv(path: Path, experiment: str, metrics: dict[str, dict]) -> None:
    fields = [
        "experiment", "signal", "samples", "rmse", "mae", "max_abs_error",
        "reference_range", "relative_rmse_percent", "dtw_mae_window_5",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for signal, values in metrics.items():
            writer.writerow({
                "experiment": experiment, "signal": signal,
                **{key: values.get(key) for key in fields[2:]},
            })

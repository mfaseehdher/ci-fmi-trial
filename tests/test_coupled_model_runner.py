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
    assert metrics["Position"]["mae"] == pytest.approx(0.15)
    assert metrics["Position"]["relative_rmse_percent"] == pytest.approx(100 * 0.025 ** 0.5 / 2.1)


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


from docs.experiments.run_coupled_models import (
    prepare_config,
    restore_sensitivity,
    rollback_reasons,
    summarize_loop_trace,
    trace_difference,
)


def coupled_config(delayed=False):
    return {
        "start": 0.0, "stop": 0.3, "dt": 0.1,
        "step_order": ["reference", "controller", "plant", "log"],
        "components": {
            "reference": {"type": "step", "ports": {}},
            "controller": {"type": "fmu", "file": "c.fmu"},
            "plant": {"type": "fmu", "file": "p.fmu"},
            "log": {"type": "logger", "file": "out.csv"},
        },
        "connections": [
            {"src_comp": "plant", "src_port": "y", "dst_comp": "controller",
             "dst_port": "y", "delayed": delayed, "default": 0.5},
            {"src_comp": "controller", "src_port": "u", "dst_comp": "plant",
             "dst_port": "u", "delayed": False, "default": 0.0},
        ],
    }


def test_prepare_config_rejects_delayed_feedback():
    with pytest.raises(ValueError, match="still delays plant.y->controller.y"):
        prepare_config(coupled_config(delayed=True))


def test_prepare_config_drops_manual_order_without_changing_input():
    original = coupled_config()
    prepared = prepare_config(original)
    assert "step_order" not in prepared
    assert "step_order" in original


def trace_entry(t, iteration, accepted):
    return {"time_start": t, "iteration": iteration, "max_residual": 0.0, "accepted": accepted}


def test_loop_trace_summary_counts_retries_per_step():
    trace = [trace_entry(0.0, 1, True),
             trace_entry(0.1, 1, False), trace_entry(0.1, 2, False), trace_entry(0.1, 3, True)]
    summary = summarize_loop_trace(trace)
    assert summary == {
        "communication_steps": 2, "accepted_steps": 2, "steps_with_retries": 1,
        "trial_steps": 4, "max_iterations": 3, "mean_iterations": 2.0,
    }


def test_trace_difference_uses_shared_times(tmp_path):
    first, second = tmp_path / "a.csv", tmp_path / "b.csv"
    write_trace(first, ["time", "y"], [(0.1, 0.0), (0.2, 4.0)])
    write_trace(second, ["time", "y"], [(0.1, 0.5), (0.2, 4.0), (0.3, 9.0)])
    assert trace_difference(first, second, ["y"]) == {"max_abs_difference": 0.5, "signal_range": 4.0}


def test_failed_no_restore_run_counts_as_detected():
    result = restore_sensitivity({"status": "error", "error": "did not converge"}, None)
    assert result["detected"] is True


def test_identical_no_restore_trace_is_not_detected():
    result = restore_sensitivity({"status": "completed"},
                                 {"max_abs_difference": 1e-9, "signal_range": 2.0})
    assert result["detected"] is False


def verified_inputs():
    probes = {"controller": {"status": "completed", "replay_identical": True, "stateful": False},
              "plant": {"status": "completed", "replay_identical": True, "stateful": True}}
    rollback = {
        "status": "completed",
        "execution_plan": [
            {"kind": "component", "components": ["reference"]},
            {"kind": "algebraic_loop", "components": ["controller", "plant"],
             "evaluation": "rollback_step"},
        ],
        "loop": {"accepted_steps": 3, "steps_with_retries": 2},
    }
    return probes, rollback, {"detected": True, "evidence": "differs"}


def test_rollback_is_verified_when_every_check_holds():
    probes, rollback, sensitivity = verified_inputs()
    assert rollback_reasons(3, probes, rollback, sensitivity) == []


@pytest.mark.parametrize("change, reason", [
    (lambda p, r, s: p["plant"].update(replay_identical=False), "different outputs"),
    (lambda p, r, s: p["plant"].update(stateful=False), "no FMU has observable state"),
    (lambda p, r, s: r["execution_plan"].pop(), "not plan the feedback loop"),
    (lambda p, r, s: r["loop"].update(steps_with_retries=0), "restore was never exercised"),
    (lambda p, r, s: r["loop"].update(accepted_steps=2), "2 of 3 steps"),
    (lambda p, r, s: s.update(detected=False), "did not change the result"),
])
def test_rollback_is_not_verified_when_a_check_fails(change, reason):
    probes, rollback, sensitivity = verified_inputs()
    change(probes, rollback, sensitivity)
    reasons = rollback_reasons(3, probes, rollback, sensitivity)
    assert any(reason in item for item in reasons), reasons


def test_probe_inputs_are_distinct_so_differences_are_not_zero():
    from docs.experiments.run_coupled_models import probe_inputs
    values = probe_inputs(["Reference", "Measured", "Rate"])
    assert len(set(values.values())) == 3


def test_every_coupled_case_has_its_setup_script():
    from docs.experiments.run_coupled_models import BASE, CASES

    assert len(set(CASES)) == len(CASES)
    for name in CASES:
        assert (BASE / name / f"setup_{name}.m").is_file(), name

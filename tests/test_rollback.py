"""Rollback-loop tests with a stateful stand-in; MATLAB FMUs are checked separately."""

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ci_fmi.master_algorithm import (
    AlgebraicLoopConvergenceError,
    Component,
    Constant,
    Logger,
    build_execution_plan,
    run_case,
    solve_rollback_loop,
)


class RollbackAdder(Component):
    def __init__(self, gain):
        super().__init__("adder")
        self.gain = gain
        self.supports_rollback = True
        self.current_time = 0.0
        self.steps = 0
        self.saved = 0
        self.restored = 0
        self.freed = 0
        self.outputs["Out"] = 0.0

    def save_state(self):
        self.saved += 1
        return self.current_time, self.steps, dict(self.inputs), dict(self.outputs)

    def restore_state(self, snapshot):
        self.restored += 1
        self.current_time, self.steps = snapshot[:2]
        self.inputs = dict(snapshot[2])
        self.outputs = dict(snapshot[3])

    def free_state(self, snapshot):
        self.freed += 1

    def trial_step(self, t, dt):
        if not math.isclose(self.current_time, t, abs_tol=1e-12):
            raise AssertionError("Trial started at the wrong time")
        self.outputs["Out"] = self.inputs["Bias"] + self.gain * self.inputs["Feedback"]
        self.current_time = round(t + dt, 12)
        self.steps += 1

    def do_step(self, t, dt):
        raise AssertionError("Accepted rollback trial must not be stepped again")


def connection(source, output, destination, input_name):
    return {
        "src_comp": source,
        "src_port": output,
        "dst_comp": destination,
        "dst_port": input_name,
        "delayed": False,
        "direct_feedthrough": True,
        "default": 0.0,
    }


class RollbackTests(unittest.TestCase):
    def test_self_loop_is_classified_for_rollback(self):
        adder = RollbackAdder(0.5)
        plan = build_execution_plan(
            {"adder": adder}, [connection("adder", "Out", "adder", "Feedback")]
        )
        self.assertEqual(plan[0]["evaluation"], "rollback_step")

    def test_converged_trial_is_accepted_once_per_step(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as directory:
            output = Path(directory) / "result.csv"
            adder = RollbackAdder(0.5)
            components = {
                "one": Constant("one", {"value": 1.0}),
                "adder": adder,
                "log": Logger("log", str(output)),
            }
            config = {
                "start": 0.0, "stop": 0.3, "dt": 0.1,
                "connections": [
                    connection("one", "value", "adder", "Bias"),
                    connection("adder", "Out", "adder", "Feedback"),
                    connection("adder", "Out", "log", "sum"),
                ],
            }
            trace = []
            with patch("ci_fmi.master_algorithm.build_components", return_value=components):
                run_case(config, loop_trace=trace)
            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(len(rows), 3)
        self.assertTrue(all(abs(float(row["sum"]) - 2.0) < 1e-6 for row in rows))
        self.assertEqual(adder.steps, 3)
        self.assertEqual(adder.saved, 3)
        self.assertEqual(adder.freed, 3)
        self.assertEqual([entry["time_start"] for entry in trace if entry["accepted"]],
                         [0.0, 0.1, 0.2])
        self.assertTrue(all(entry["trial_start_times"]["adder"] == entry["time_start"]
                            for entry in trace))

    def test_nonconvergence_restores_state(self):
        adder = RollbackAdder(1.0)
        adder.set_input("Bias", 1.0)
        components = {"adder": adder}
        links = [connection("adder", "Out", "adder", "Feedback")]
        with self.assertRaises(AlgebraicLoopConvergenceError):
            solve_rollback_loop(["adder"], components, links, 0.0, 0.1,
                                max_iterations=5)
        self.assertEqual(adder.current_time, 0.0)
        self.assertEqual(adder.steps, 0)
        self.assertEqual(adder.outputs["Out"], 0.0)
        self.assertEqual(adder.saved, 1)
        self.assertEqual(adder.freed, 1)


if __name__ == "__main__":
    unittest.main()

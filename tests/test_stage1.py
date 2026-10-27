"""Algorithm tests use Python stand-ins, not MATLAB FMUs."""

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
    AlgebraicLoopError,
    Component,
    FMUComponent,
    Logger,
    build_execution_plan,
    run_case,
    solve_algebraic_loop,
)


class Source(Component):
    def __init__(self, sine=False):
        super().__init__("source")
        self.sine = sine

    def do_step(self, t, dt):
        self.outputs["r"] = 1.0 + math.sin(t) if self.sine else 1.0


class Affine(Component):
    def __init__(self, name, gain):
        super().__init__(name)
        self.gain = gain
        self.outputs["y"] = 0.0
        self.steps = 0

    def evaluate_outputs(self):
        self.outputs["y"] = self.inputs.get("bias", 0.0) + self.gain * self.inputs.get("u", 0.0)

    def do_step(self, t, dt):
        self.evaluate_outputs()
        self.steps += 1


def link(source, port, destination, input_port, delayed=False):
    return {
        "src_comp": source, "src_port": port,
        "dst_comp": destination, "dst_port": input_port,
        "delayed": delayed, "direct_feedthrough": True, "default": 0.0,
    }


def loop_connections(delayed=False):
    return [
        link("source", "r", "a", "bias"),
        link("b", "y", "a", "u", delayed),
        link("a", "y", "b", "u"),
        link("a", "y", "log", "sum"),
    ]


class StageOneTests(unittest.TestCase):
    def run_with_stand_ins(self, delayed=False, sine=False, stop=0.3):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as directory:
            csv_path = Path(directory) / "result.csv"
            components = {
                "source": Source(sine), "a": Affine("a", 0.5),
                "b": Affine("b", 0.2), "log": Logger("log", str(csv_path)),
            }
            config = {
                "start": 0.0, "stop": stop, "dt": 0.1,
                "connections": loop_connections(delayed),
            }
            with patch("ci_fmi.master_algorithm.build_components", return_value=components):
                run_case(config)
            with csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        return rows, components

    def test_chain_without_loop(self):
        components = {name: Affine(name, 1.0) for name in ("a", "b", "c")}
        plan = build_execution_plan(components, [
            link("a", "y", "b", "u"), link("b", "y", "c", "u"),
        ])
        self.assertEqual([stage["components"] for stage in plan], [["a"], ["b"], ["c"]])

    def test_manual_order_cannot_reverse_dependency(self):
        components = {"a": Affine("a", 1.0), "b": Affine("b", 1.0)}
        config = {
            "start": 0.0, "stop": 0.1, "dt": 0.1,
            "connections": [link("a", "y", "b", "u")],
            "step_order": ["b", "a"],
        }
        with patch("ci_fmi.master_algorithm.build_components", return_value=components):
            with self.assertRaisesRegex(ValueError, "violates a same-time connection"):
                run_case(config)

    def test_convergent_loop_on_first_step(self):
        rows, components = self.run_with_stand_ins(stop=0.1)
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(float(rows[0]["sum"]), 1.0 / 0.9, places=6)
        self.assertEqual(components["a"].steps, 1)
        self.assertEqual(components["b"].steps, 1)

    def test_explicit_delay_differs(self):
        loop_rows, _ = self.run_with_stand_ins(stop=0.2)
        delayed_rows, _ = self.run_with_stand_ins(delayed=True, stop=0.2)
        self.assertAlmostEqual(float(loop_rows[0]["sum"]), 1.0 / 0.9, places=6)
        self.assertEqual([float(row["sum"]) for row in delayed_rows], [1.0, 1.1])

    def test_loop_solved_every_step(self):
        rows, _ = self.run_with_stand_ins(sine=True)
        self.assertEqual(len(rows), 3)
        for index, row in enumerate(rows):
            expected = (1.0 + math.sin(index * 0.1)) / 0.9
            self.assertAlmostEqual(float(row["sum"]), expected, places=6)

    def test_loop_without_solution(self):
        components = {"a": Affine("a", 1.0), "b": Affine("b", 1.0)}
        components["a"].inputs["bias"] = 1.0
        connections = [link("a", "y", "b", "u"), link("b", "y", "a", "u")]
        with self.assertRaises(AlgebraicLoopConvergenceError):
            solve_algebraic_loop(["a", "b"], components, connections, max_iterations=8)

    def test_no_automatic_delay(self):
        components = {"a": Affine("a", 0.5), "b": Affine("b", 0.2)}
        connections = [link("a", "y", "b", "u"), link("b", "y", "a", "u")]
        plan = build_execution_plan(components, connections)
        self.assertEqual(plan[0]["kind"], "algebraic_loop")
        self.assertFalse(any(connection["delayed"] for connection in connections))

    def test_do_step_only_fmu_rejected(self):
        components = {
            "a": FMUComponent("a", "not_loaded.fmu"),
            "b": FMUComponent("b", "not_loaded.fmu"),
        }
        connections = [link("a", "y", "b", "u"), link("b", "y", "a", "u")]
        with self.assertRaisesRegex(AlgebraicLoopError, "verified same-time"):
            build_execution_plan(components, connections)
        self.assertFalse(any(connection["delayed"] for connection in connections))


if __name__ == "__main__":
    unittest.main()

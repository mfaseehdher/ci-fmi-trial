"""Step operation graph (Hansen et al., Def. 4) and loop classification.

Python stand-ins replace MATLAB FMUs, so these tests need no MATLAB.
"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ci_fmi.master_algorithm import (
    AlgebraicLoopError,
    Component,
    Constant,
    Logger,
    build_execution_plan,
    build_step_operation_graph,
    run_case,
)


def link(source, output, destination, input_name, *, delayed=False, feedthrough=None):
    return {
        "src_comp": source, "src_port": output,
        "dst_comp": destination, "dst_port": input_name,
        "delayed": delayed, "direct_feedthrough": feedthrough, "default": 0.0,
    }


class Rollback(Component):
    """Stand-in with FMI-like save/restore around a stepping law."""

    def __init__(self, name):
        super().__init__(name)
        self.supports_rollback = True
        self.current_time = 0.0
        self.state = 0.0

    def save_state(self):
        return self.current_time, self.state, dict(self.inputs), dict(self.outputs)

    def restore_state(self, snapshot):
        self.current_time, self.state = snapshot[:2]
        self.inputs = dict(snapshot[2])
        self.outputs = dict(snapshot[3])

    def free_state(self, snapshot):
        pass

    def trial_step(self, t, dt):
        self.do_step(t, dt)


class Controller(Rollback):
    """Proportional controller: force = k * (reference - speed), feeds through."""

    def __init__(self, gain):
        super().__init__("controller")
        self.gain = gain
        self.outputs["force"] = 0.0

    def do_step(self, t, dt):
        error = self.inputs.get("reference", 0.0) - self.inputs.get("speed", 0.0)
        self.outputs["force"] = self.gain * error
        self.current_time = round(t + dt, 12)


class Integrator(Rollback):
    """speed(t+dt) = speed(t) + dt * force: no direct feedthrough."""

    def __init__(self):
        super().__init__("plant")
        self.outputs["speed"] = 0.0

    def do_step(self, t, dt):
        self.state += dt * self.inputs.get("force", 0.0)
        self.outputs["speed"] = self.state
        self.current_time = round(t + dt, 12)


def closed_loop(delayed=False):
    return [
        link("reference", "r", "controller", "reference"),
        link("plant", "speed", "controller", "speed", delayed=delayed, feedthrough=True),
        link("controller", "force", "plant", "force", feedthrough=False),
        link("plant", "speed", "log", "speed"),
    ]


class StepOperationGraphTests(unittest.TestCase):
    def test_edges_follow_the_five_rules(self):
        components = {"a": Component("a"), "b": Component("b")}
        graph = build_step_operation_graph(components, [
            link("a", "y", "b", "u", feedthrough=True),
            link("b", "z", "a", "w", delayed=True, feedthrough=False),
        ])
        rules = {(source, target): data["rule"] for source, target, data in graph.edges(data=True)}
        self.assertEqual(rules[(("get", "a", "y"), ("set", "b", "u"))], 1)
        self.assertEqual(rules[(("step", "a"), ("get", "a", "y"))], 2)
        self.assertEqual(rules[(("set", "b", "u"), ("step", "b"))], 3)
        self.assertEqual(rules[(("step", "a"), ("set", "a", "w"))], 4)
        self.assertEqual(rules[(("set", "b", "u"), ("get", "b", "z"))], 5)
        # A delayed connection carries last step's value: no rule-1 edge,
        # and no rule-5 edge where the user declared no feedthrough.
        self.assertNotIn((("get", "b", "z"), ("set", "a", "w")), rules)
        self.assertNotIn((("set", "a", "w"), ("get", "a", "y")), rules)

    def test_integrator_loop_is_a_reactivity_loop_solved_by_rollback(self):
        components = {
            "reference": Constant("reference", {"r": 1.0}),
            "controller": Controller(2.0), "plant": Integrator(),
            "log": Logger("log", "unused.csv"),
        }
        plan = build_execution_plan(components, closed_loop())
        self.assertEqual(plan, [
            {"kind": "component", "components": ["reference"]},
            {"kind": "algebraic_loop", "components": ["controller", "plant"],
             "loop_type": "reactivity", "evaluation": "rollback_step"},
            {"kind": "component", "components": ["log"]},
        ])

    def test_reactivity_loop_without_rollback_is_rejected(self):
        components = {
            "controller": Component("controller"), "plant": Component("plant"),
        }
        with self.assertRaisesRegex(AlgebraicLoopError, "needs FMU state rollback"):
            build_execution_plan(components, [
                link("plant", "speed", "controller", "speed", feedthrough=True),
                link("controller", "force", "plant", "force", feedthrough=False),
            ])

    def test_pure_feedthrough_loop_is_a_feedthrough_loop(self):
        components = {"a": Component("a"), "b": Component("b")}
        plan = build_execution_plan(components, [
            link("a", "y", "b", "u", feedthrough=True),
            link("b", "y", "a", "u", feedthrough=True),
        ])
        self.assertEqual(plan[0]["loop_type"], "feedthrough")
        self.assertEqual(plan[0]["evaluation"], "same_time")

    def test_output_outside_the_loop_does_not_join_it(self):
        # c reads b's second output but feeds nothing back, so it runs after
        # the loop instead of joining it.
        components = {"a": Component("a"), "b": Component("b"), "c": Component("c")}
        plan = build_execution_plan(components, [
            link("a", "y", "b", "u", feedthrough=True),
            link("b", "y1", "a", "u", feedthrough=True),
            link("b", "y2", "c", "u", feedthrough=True),
        ])
        self.assertEqual(plan[0]["components"], ["a", "b"])
        self.assertEqual(plan[0]["loop_type"], "feedthrough")
        self.assertEqual(plan[1], {"kind": "component", "components": ["c"]})

    def test_missing_feedthrough_in_loop_is_reported(self):
        components = {"a": Component("a"), "b": Component("b")}
        with self.assertRaisesRegex(AlgebraicLoopError, "information is missing"):
            build_execution_plan(components, [
                link("a", "y", "b", "u"), link("b", "y", "a", "u"),
            ])

    def test_delayed_feedback_breaks_the_loop(self):
        components = {
            "reference": Constant("reference", {"r": 1.0}),
            "controller": Controller(2.0), "plant": Integrator(),
            "log": Logger("log", "unused.csv"),
        }
        plan = build_execution_plan(components, closed_loop(delayed=True))
        self.assertEqual([stage["kind"] for stage in plan], ["component"] * 4)

    def test_closed_loop_matches_implicit_euler(self):
        # With the loop solved every step: x1 = x0 + dt*k*(r - x1).
        gain, dt, steps = 2.0, 0.1, 5
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as directory:
            path = Path(directory) / "result.csv"
            components = {
                "reference": Constant("reference", {"r": 1.0}),
                "controller": Controller(gain), "plant": Integrator(),
                "log": Logger("log", str(path)),
            }
            config = {"start": 0.0, "stop": dt * steps, "dt": dt, "connections": closed_loop()}
            with patch("ci_fmi.master_algorithm.build_components", return_value=components):
                run_case(config)
            with path.open(newline="", encoding="utf-8") as handle:
                speeds = [float(row["speed"]) for row in csv.DictReader(handle)]

        expected, x = [], 0.0
        for _ in range(steps):
            x = (x + dt * gain * 1.0) / (1.0 + dt * gain)
            expected.append(x)
        self.assertEqual(len(speeds), steps)
        for actual, wanted in zip(speeds, expected):
            self.assertAlmostEqual(actual, wanted, places=7)


if __name__ == "__main__":
    unittest.main()

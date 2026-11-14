"""Initialization graph and fixed-point initialization (Hansen et al. 2021).

Python stand-ins replace MATLAB FMUs, so these tests need no MATLAB.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ci_fmi.master_algorithm import (
    AlgebraicLoopConvergenceError,
    AlgebraicLoopError,
    Component,
    Constant,
    FMUComponent,
    StepSignal,
    build_initialization_graph,
    initialize_co_simulation,
)


def link(source, output, destination, input_name, *, delayed=False, feedthrough=True):
    return {
        "src_comp": source, "src_port": output,
        "dst_comp": destination, "dst_port": input_name,
        "delayed": delayed, "direct_feedthrough": feedthrough, "default": 0.0,
    }


class Affine(Component):
    """y = bias + gain * u, evaluated without stepping."""

    def __init__(self, name, gain):
        super().__init__(name)
        self.gain = gain
        self.outputs["y"] = 0.0

    def evaluate_outputs(self):
        self.outputs["y"] = self.inputs.get("bias", 0.0) + self.gain * self.inputs.get("u", 0.0)


class MatlabLikeSlave:
    """Fake FMI 2.0 slave that behaves like the MATLAB-exported FMUs.

    Outputs are computed only in exitInitializationMode; setting an input
    in initialization mode does not change them. Restoring a state saved in
    initialization mode returns the slave to initialization mode.
    """

    def __init__(self, law, inputs, outputs):
        self.law = law
        self.values = {name: 0.0 for name in inputs + outputs}
        self.outputs = outputs
        self.mode = "initialization"
        self.exits = 0

    def setReal(self, references, values):
        for reference, value in zip(references, values):
            self.values[reference] = value

    def getReal(self, references):
        return [self.values[reference] for reference in references]

    def exitInitializationMode(self):
        if self.mode != "initialization":
            raise RuntimeError("not in initialization mode")
        self.values.update(self.law(self.values))
        self.mode = "initialized"
        self.exits += 1

    def getFMUstate(self):
        return self.mode, dict(self.values)

    def setFMUstate(self, state):
        self.mode, values = state
        self.values = dict(values)

    def freeFMUstate(self, state):
        pass


def matlab_like(name, law, inputs, outputs, *, rollback=True):
    component = FMUComponent(name, f"{name}.fmu")
    component.fmu = MatlabLikeSlave(law, inputs, outputs)
    component.vr = {port: port for port in inputs + outputs}
    component.var_types = {port: "Real" for port in inputs + outputs}
    component.inputs_list = list(inputs)
    component.outputs_list = list(outputs)
    # MATLAB omits the dependency list: every output depends on every input.
    component.initial_dependencies = {port: set(inputs) for port in outputs}
    component.feedthrough_inputs = dict(component.initial_dependencies)
    component.supports_rollback = rollback
    component.in_initialization = True
    component.current_time = 0.0
    return component


def controller(rollback=True):
    return matlab_like(
        "controller",
        lambda v: {"Force": 800.0 * (v["ReferenceVelocity"] - v["MeasuredVelocity"])},
        ["ReferenceVelocity", "MeasuredVelocity"], ["Force"], rollback=rollback,
    )


def plant():
    # The integrator's initial condition: velocity starts at 0 whatever the force.
    return matlab_like("plant", lambda v: {"Velocity": 0.0}, ["Force"], ["Velocity"])


def cruise_connections():
    return [
        link("reference", "r", "controller", "ReferenceVelocity", feedthrough=None),
        link("plant", "Velocity", "controller", "MeasuredVelocity", feedthrough=None),
        link("controller", "Force", "plant", "Force", feedthrough=None),
    ]


class InitializationGraphTests(unittest.TestCase):
    def test_edges_follow_definition_8(self):
        components = {"a": Affine("a", 1.0), "b": Affine("b", 1.0)}
        graph = build_initialization_graph(components, [
            link("a", "y", "b", "u"),
            link("b", "y", "a", "u", delayed=True),
            link("a", "y", "b", "bias", feedthrough=False),
        ])
        edges = set(graph.edges)
        self.assertIn((("out", "a", "y"), ("in", "b", "u")), edges)  # rule 1
        self.assertIn((("in", "b", "u"), ("out", "b", "y")), edges)  # rule 2
        self.assertNotIn((("in", "b", "bias"), ("out", "b", "y")), edges)
        # The delayed link takes the delay's start value instead.
        self.assertNotIn((("out", "b", "y"), ("in", "a", "u")), edges)


class InitializationTests(unittest.TestCase):
    def test_chain_is_initialized_in_order(self):
        components = {
            "source": Constant("source", {"r": 2.0}),
            "a": Affine("a", 3.0), "b": Affine("b", 0.5),
        }
        report = initialize_co_simulation(components, [
            link("source", "r", "a", "u"), link("a", "y", "b", "u"),
        ], 0.0)
        self.assertEqual(report["loops"], [])
        self.assertEqual(components["a"].outputs["y"], 6.0)
        self.assertEqual(components["b"].outputs["y"], 3.0)

    def test_loop_reaches_its_fixed_point_before_the_first_step(self):
        components = {
            "source": Constant("source", {"r": 1.0}),
            "a": Affine("a", 0.5), "b": Affine("b", 0.2),
        }
        report = initialize_co_simulation(components, [
            link("source", "r", "a", "bias"),
            link("b", "y", "a", "u"), link("a", "y", "b", "u"),
        ], 0.0)
        self.assertAlmostEqual(components["a"].outputs["y"], 1.0 / 0.9, places=6)
        self.assertAlmostEqual(components["b"].outputs["y"], 0.2 / 0.9, places=6)
        self.assertAlmostEqual(components["a"].inputs["u"], components["b"].outputs["y"], places=6)
        self.assertEqual(len(report["loops"]), 1)
        self.assertEqual(report["loops"][0]["outputs"], ["a.y", "b.y"])

    def test_user_guess_is_the_first_iterate(self):
        components = {"a": Affine("a", 0.5), "b": Affine("b", 0.2)}
        report = initialize_co_simulation(components, [
            link("b", "y", "a", "u"), link("a", "y", "b", "u"),
        ], 0.0, initial_guesses={"a.y": 4.0})
        loop = report["loops"][0]
        self.assertEqual(loop["first_guess"]["a.y"], 4.0)
        self.assertEqual(loop["guess_source"]["a.y"], "initial_values")
        self.assertAlmostEqual(loop["values"]["a.y"], 0.0, places=6)

    def test_loop_without_solution_stops_the_run(self):
        components = {"source": Constant("source", {"r": 1.0}), "a": Affine("a", 1.0)}
        with self.assertRaisesRegex(AlgebraicLoopConvergenceError, "did not converge"):
            initialize_co_simulation(components, [
                link("source", "r", "a", "bias"), link("a", "y", "a", "u"),
            ], 0.0, max_iterations=10)

    def test_sources_give_their_value_at_the_start_time(self):
        components = {
            "step": StepSignal("step", {"r": {"initial": 0.0, "step": 10.0, "step_time": 0.0}}),
            "a": Affine("a", 1.0),
        }
        initialize_co_simulation(components, [link("step", "r", "a", "u")], 0.0)
        self.assertEqual(components["a"].outputs["y"], 10.0)


class MatlabLikeFMUTests(unittest.TestCase):
    def test_cruise_loop_is_initialized_by_restoring_initialization_mode(self):
        components = {
            "reference": Constant("reference", {"r": 10.0}),
            "controller": controller(), "plant": plant(),
        }
        report = initialize_co_simulation(components, cruise_connections(), 0.0)
        # Velocity starts at 0, so the consistent start force is 800 * 10.
        self.assertEqual(components["controller"].outputs["Force"], 8000.0)
        self.assertEqual(components["plant"].outputs["Velocity"], 0.0)
        self.assertEqual(components["plant"].inputs["Force"], 8000.0)
        self.assertEqual(report["loops"][0]["iterations"], 2)
        for name in ("controller", "plant"):
            self.assertFalse(components[name].in_initialization)
            self.assertIsNone(components[name]._init_snapshot)
            self.assertEqual(components[name].fmu.mode, "initialized")

    def test_loop_member_without_state_support_is_rejected(self):
        components = {
            "reference": Constant("reference", {"r": 10.0}),
            "controller": controller(rollback=False), "plant": plant(),
        }
        with self.assertRaisesRegex(AlgebraicLoopError, "repeated evaluation.*controller"):
            initialize_co_simulation(components, cruise_connections(), 0.0)

    def test_fmu_outside_a_loop_leaves_initialization_mode_once(self):
        components = {
            "reference": Constant("reference", {"r": 10.0}),
            "controller": controller(rollback=False),
        }
        initialize_co_simulation(components, [
            link("reference", "r", "controller", "ReferenceVelocity", feedthrough=None),
        ], 0.0)
        self.assertEqual(components["controller"].fmu.exits, 1)
        self.assertEqual(components["controller"].outputs["Force"], 8000.0)


if __name__ == "__main__":
    unittest.main()

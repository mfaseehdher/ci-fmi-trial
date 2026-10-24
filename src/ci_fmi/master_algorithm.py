"""
generic.py  —  FMI 2.0 co-simulation master algorithm
Usage: python master_algorithm.py <case.json>

This stage solves same-time loops only when outputs can be evaluated without
calling doStep during iteration. It does not implement FMU state rollback.

Key feature: automatic sub-stepping for fixed-step MATLAB/Simulink FMUs.
The FMU's compiled step size is read from DefaultExperiment.stepSize and used
as the internal step. The master loop's dt must be a multiple of that step.
"""

import csv
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Tuple

try:
    from fmpy import extract, read_model_description
    from fmpy.fmi2 import FMU2Slave
    FMPY_AVAILABLE = True
except ImportError:
    FMPY_AVAILABLE = False

try:
    import networkx as nx
    NETWORKX_AVAILABLE = True
except ImportError:
    NETWORKX_AVAILABLE = False


# ──────────────────────────────────────────────────────────────
# Base component
# ──────────────────────────────────────────────────────────────

class Component:
    def __init__(self, name: str) -> None:
        self.name = name
        self.inputs:  Dict[str, Any] = {}
        self.outputs: Dict[str, Any] = {}

    def set_input(self, key: str, value: Any) -> None:
        self.inputs[key] = value

    def get_output(self, key: str) -> Any:
        return self.outputs.get(key, 0.0)

    def do_step(self, t: float, dt: float) -> None:
        pass

    def evaluate_outputs(self) -> None:
        """Re-evaluate outputs at the current time without advancing time."""
        raise NotImplementedError(
            f"Component '{self.name}' does not support same-time output evaluation"
        )

    def terminate(self) -> None:
        pass


# ──────────────────────────────────────────────────────────────
# CSV Stimulus
# ──────────────────────────────────────────────────────────────

class CSVStimulus(Component):
    """
    Reads a CSV and provides signal values each step.
    - If a 'time' column exists: holds the last row whose time <= t.
    - Otherwise: advances one row per do_step call.
    """

    def __init__(self, name: str, path: str, time_col: str = "time") -> None:
        super().__init__(name)
        self.time_col = time_col

        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            self.fields: List[str] = list(reader.fieldnames or [])
            self.rows: List[Dict[str, Any]] = []
            for row in reader:
                parsed: Dict[str, Any] = {}
                for k, v in row.items():
                    if v is None or str(v).strip() == "":
                        continue
                    parsed[k] = _cast(v)
                self.rows.append(parsed)

        if not self.rows:
            raise ValueError(f"CSVStimulus '{name}': '{path}' is empty")

        self.has_time = self.time_col in self.fields
        self.i = 0

    def signal_names(self) -> List[str]:
        return [f for f in self.fields if f != self.time_col]

    def do_step(self, t: float, dt: float) -> None:
        if self.has_time:
            while (self.i < len(self.rows) - 1 and
                   float(self.rows[self.i + 1][self.time_col]) <= t + 1e-12):
                self.i += 1
            row = self.rows[self.i]
        else:
            row = self.rows[min(self.i, len(self.rows) - 1)]
            self.i += 1

        for k, v in row.items():
            if k != self.time_col:
                self.outputs[k] = v


# ──────────────────────────────────────────────────────────────
# Constant source
# ──────────────────────────────────────────────────────────────

class Constant(Component):
    def __init__(self, name: str, values: Dict[str, Any]) -> None:
        super().__init__(name)
        self._values = {k: _cast(str(v)) for k, v in values.items()}

    def do_step(self, t: float, dt: float) -> None:
        self.outputs.update(self._values)


# ──────────────────────────────────────────────────────────────
# Signal generator components (alternative to CSV input)
# ──────────────────────────────────────────────────────────────
# These let you describe experiments declaratively in JSON instead of
# pre-computing CSV files. Each component produces one or more output
# signals as a function of time.
#
# Example JSON:
#   "stim": {
#     "type": "step",
#     "ports": {
#       "Throttle": {"initial": 0.0, "step": 500.0, "step_time": 0.0}
#     }
#   }


class StepSignal(Component):
    """
    Generates step signals on one or more output ports.

    Mirrors Simulink's Step block. Each port has:
      initial   : value before step_time
      step      : value at and after step_time
      step_time : time of the step (seconds)
    """
    def __init__(self, name: str, ports: Dict[str, Dict[str, Any]]) -> None:
        super().__init__(name)
        self._ports = {}
        for port, spec in ports.items():
            self._ports[port] = {
                "initial":   float(spec.get("initial",   0.0)),
                "step":      float(spec.get("step",      1.0)),
                "step_time": float(spec.get("step_time", 0.0)),
            }

    def do_step(self, t: float, dt: float) -> None:
        for port, p in self._ports.items():
            self.outputs[port] = p["step"] if t >= p["step_time"] else p["initial"]


class SineSignal(Component):
    """
    Generates sinusoidal signals on one or more output ports.

    Mirrors Simulink's Sine Wave block. Each port has:
      amplitude : peak amplitude
      frequency : in Hz (cycles per second)
      phase     : phase offset in radians
      bias      : DC offset
    """
    def __init__(self, name: str, ports: Dict[str, Dict[str, Any]]) -> None:
        super().__init__(name)
        self._ports = {}
        for port, spec in ports.items():
            self._ports[port] = {
                "amplitude": float(spec.get("amplitude", 1.0)),
                "frequency": float(spec.get("frequency", 1.0)),
                "phase":     float(spec.get("phase",     0.0)),
                "bias":      float(spec.get("bias",      0.0)),
            }

    def do_step(self, t: float, dt: float) -> None:
        import math
        for port, p in self._ports.items():
            self.outputs[port] = (
                p["bias"]
                + p["amplitude"]
                * math.sin(2.0 * math.pi * p["frequency"] * t + p["phase"])
            )


class RampSignal(Component):
    """
    Generates ramp signals on one or more output ports.

    Mirrors Simulink's Ramp block. Each port has:
      initial    : value before start_time
      slope      : rate of change (units per second)
      start_time : time at which ramp begins (seconds)
    """
    def __init__(self, name: str, ports: Dict[str, Dict[str, Any]]) -> None:
        super().__init__(name)
        self._ports = {}
        for port, spec in ports.items():
            self._ports[port] = {
                "initial":    float(spec.get("initial",    0.0)),
                "slope":      float(spec.get("slope",      1.0)),
                "start_time": float(spec.get("start_time", 0.0)),
            }

    def do_step(self, t: float, dt: float) -> None:
        for port, p in self._ports.items():
            if t < p["start_time"]:
                self.outputs[port] = p["initial"]
            else:
                self.outputs[port] = (
                    p["initial"] + p["slope"] * (t - p["start_time"])
                )


class PulseSignal(Component):
    """
    Generates pulse train signals (square wave) on one or more output ports.

    Mirrors Simulink's Pulse Generator block. Each port has:
      amplitude   : pulse height
      period      : pulse period (seconds)
      duty_cycle  : fraction of period the pulse is high (0..1)
      phase_delay : initial delay (seconds)
      bias        : DC offset (value when pulse is low)
    """
    def __init__(self, name: str, ports: Dict[str, Dict[str, Any]]) -> None:
        super().__init__(name)
        self._ports = {}
        for port, spec in ports.items():
            self._ports[port] = {
                "amplitude":   float(spec.get("amplitude",   1.0)),
                "period":      float(spec.get("period",      1.0)),
                "duty_cycle":  float(spec.get("duty_cycle",  0.5)),
                "phase_delay": float(spec.get("phase_delay", 0.0)),
                "bias":        float(spec.get("bias",        0.0)),
            }

    def do_step(self, t: float, dt: float) -> None:
        for port, p in self._ports.items():
            if t < p["phase_delay"]:
                self.outputs[port] = p["bias"]
                continue
            local_t = (t - p["phase_delay"]) % p["period"]
            if local_t < p["duty_cycle"] * p["period"]:
                self.outputs[port] = p["bias"] + p["amplitude"]
            else:
                self.outputs[port] = p["bias"]


# ──────────────────────────────────────────────────────────────
# FMU component
# ──────────────────────────────────────────────────────────────

_SETTERS = {
    "Real":    lambda fmu, vr, v: fmu.setReal(vr, [float(v)]),
    "Integer": lambda fmu, vr, v: fmu.setInteger(vr, [int(round(float(v)))]),
    "Boolean": lambda fmu, vr, v: fmu.setBoolean(vr, [bool(int(float(v)))]),
    "String":  lambda fmu, vr, v: fmu.setString(vr, [str(v)]),
}
_GETTERS = {
    "Real":    lambda fmu, vr: fmu.getReal(vr)[0],
    "Integer": lambda fmu, vr: fmu.getInteger(vr)[0],
    "Boolean": lambda fmu, vr: fmu.getBoolean(vr)[0],
    "String":  lambda fmu, vr: fmu.getString(vr)[0],
}


class FMUComponent(Component):
    """
    Wraps an FMI 2.0 Co-Simulation FMU with automatic sub-stepping.

    Sub-stepping logic
    ------------------
    MATLAB Simulink FMUs compiled with a fixed-step solver enforce:
        "Stepsize must be divisible by <compiled_step>"
    So you CANNOT call doStep with a step smaller than the compiled step.

    The correct approach:
        1. Read declared_step from DefaultExperiment.stepSize
        2. Call doStep exactly (dt / declared_step) times per comm step
        3. dt must be an integer multiple of declared_step

    configure_stepping(dt) handles all of this automatically.
    """

    def __init__(
        self, name: str, fmu_path: str, same_time_evaluation: bool = False
    ) -> None:
        if not FMPY_AVAILABLE:
            raise ImportError("Install fmpy:  pip install fmpy")
        super().__init__(name)
        self.fmu_path = fmu_path
        self.same_time_evaluation = same_time_evaluation
        self.fmu: Any = None
        self.vr:              Dict[str, int] = {}
        self.var_types:       Dict[str, str] = {}
        self.inputs_list:     List[str] = []
        self.outputs_list:    List[str] = []
        self.parameters_list: List[str] = []
        self.start_values:    Dict[str, float] = {}  # initial values from FMU XML
        self.feedthrough_inputs: Dict[str, Optional[set[str]]] = {}
        self.declared_step:   Optional[float] = None
        self._n_sub:          int   = 1     # sub-steps per communication step
        self._h:              float = 0.0   # step size passed to doStep

    def instantiate(self, start: float,
                    params: Optional[Dict[str, Any]] = None) -> None:
        md = read_model_description(self.fmu_path, validate=False)
        unzipdir = extract(self.fmu_path)

        # Read the FMU's compiled fixed step size
        de = getattr(md, "defaultExperiment", None)
        if de is not None:
            raw = getattr(de, "stepSize", None)
            if raw is not None:
                try:
                    self.declared_step = float(raw)
                except (TypeError, ValueError):
                    pass

        for v in md.modelVariables:
            self.vr[v.name] = v.valueReference
            self.var_types[v.name] = v.type if hasattr(v, "type") else "Real"
            # Read declared start value from XML (set by MATLAB Integrator's
            # "Initial condition" parameter or equivalent). This is what
            # MATLAB stores when you export the FMU.
            start_val = getattr(v, "start", None)
            if start_val is not None:
                try:
                    self.start_values[v.name] = float(start_val)
                except (TypeError, ValueError):
                    pass
            if v.causality == "input":
                self.inputs_list.append(v.name)
            elif v.causality == "output":
                self.outputs_list.append(v.name)
            elif (v.causality in ("parameter", "calculatedParameter") or
                  getattr(v, "variability", "") == "tunable"):
                self.parameters_list.append(v.name)

        # FMPy maps both an omitted dependencies attribute and an explicitly
        # empty one to []. FMI 2.0 gives those cases opposite meanings, so read
        # this small part of modelDescription.xml directly.
        xml_root = ET.parse(
            os.path.join(unzipdir, "modelDescription.xml")
        ).getroot()
        xml_variables = xml_root.findall("./ModelVariables/ScalarVariable")
        input_names = set(self.inputs_list)
        for unknown in xml_root.findall("./ModelStructure/Outputs/Unknown"):
            output_index = int(unknown.attrib["index"]) - 1
            output_name = xml_variables[output_index].attrib["name"]
            raw_dependencies = unknown.get("dependencies")
            if raw_dependencies is None:
                # FMI 2.0: omitted means dependency on every known input.
                self.feedthrough_inputs[output_name] = set(input_names)
                continue
            dependency_names = {
                xml_variables[int(index) - 1].attrib["name"]
                for index in raw_dependencies.split()
            }
            self.feedthrough_inputs[output_name] = (
                dependency_names & input_names
            )

        self.fmu = FMU2Slave(
            guid=md.guid,
            unzipDirectory=unzipdir,
            modelIdentifier=md.coSimulation.modelIdentifier,
            instanceName=self.name,
        )
        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=start)
        self.fmu.enterInitializationMode()

        if params:
            for k, v in params.items():
                self._fmu_set(k, v)

        self.fmu.exitInitializationMode()

    def configure_stepping(self, comm_dt: float) -> None:
        """
        Must be called once after instantiate(), before the master loop.
        Determines _n_sub and _h so that:
            _n_sub * _h == comm_dt   (exactly)
            _h == declared_step      (FMU constraint)
        """
        if self.declared_step and self.declared_step > 0:
            n = int(round(comm_dt / self.declared_step))
            n = max(1, n)

            # Hard check: dt must be a multiple of declared_step
            error = abs(n * self.declared_step - comm_dt)
            if error / comm_dt > 0.001:
                raise ValueError(
                    f"\n[FMU '{self.name}'] dt={comm_dt} is NOT a multiple of "
                    f"declared stepSize={self.declared_step}.\n"
                    f"  Fix: change 'dt' in your JSON to a multiple of "
                    f"{self.declared_step}.\n"
                    f"  E.g. dt={self.declared_step} or dt="
                    f"{self.declared_step * 10} etc."
                )

            self._n_sub = n
            self._h = self.declared_step   # always the exact compiled step

        else:
            # No declared step info — single call per communication step
            self._n_sub = 1
            self._h = comm_dt

    def do_step(self, t: float, dt: float) -> None:
        # Set inputs (held constant across all sub-steps — ZOH)
        for k, v in self.inputs.items():
            self._fmu_set(k, v)

        # Sub-step loop
        t_local = t
        for _ in range(self._n_sub):
            self.fmu.doStep(t_local, self._h)
            t_local = round(t_local + self._h, 12)

        # Read outputs after the full communication step
        for k in self.outputs_list:
            self.outputs[k] = self._fmu_get(k)

    def evaluate_outputs(self) -> None:
        """Apply inputs and read direct-feedthrough outputs without doStep."""
        for key, value in self.inputs.items():
            self._fmu_set(key, value)
        for key in self.outputs_list:
            self.outputs[key] = self._fmu_get(key)

    def has_direct_feedthrough(
        self, input_name: str, output_name: str
    ) -> Optional[bool]:
        dependencies = self.feedthrough_inputs.get(output_name)
        if dependencies is None:
            return None
        return input_name in dependencies

    def list_ports(self) -> Dict[str, List[str]]:
        return {
            "inputs":     self.inputs_list,
            "outputs":    self.outputs_list,
            "parameters": self.parameters_list,
        }

    def _fmu_set(self, name: str, value: Any) -> None:
        if name not in self.vr:
            raise KeyError(
                f"FMU '{self.name}': variable '{name}' not found.\n"
                f"  Available variables: {list(self.vr.keys())}"
            )
        vtype  = self.var_types.get(name, "Real")
        setter = _SETTERS.get(vtype)
        if setter is None:
            raise ValueError(
                f"FMU '{self.name}': unsupported type '{vtype}' for '{name}'"
            )
        setter(self.fmu, [self.vr[name]], value)

    def _fmu_get(self, name: str) -> Any:
        if name not in self.vr:
            raise KeyError(f"FMU '{self.name}': variable '{name}' not found")
        vtype  = self.var_types.get(name, "Real")
        getter = _GETTERS.get(vtype)
        if getter is None:
            raise ValueError(
                f"FMU '{self.name}': unsupported type '{vtype}' for '{name}'"
            )
        return getter(self.fmu, [self.vr[name]])

    def terminate(self) -> None:
        if self.fmu is not None:
            self.fmu.terminate()
            self.fmu.freeInstance()


# ──────────────────────────────────────────────────────────────
# Logger
# ──────────────────────────────────────────────────────────────

class Logger(Component):
    def __init__(self, name: str, path: str) -> None:
        super().__init__(name)
        self.path = path
        self.rows: List[Dict[str, Any]] = []

    def do_step(self, t: float, dt: float) -> None:
        # Record time as t+dt (end of communication step).
        # FMU outputs represent the state AFTER doStep(t, dt) completes,
        # i.e. the state at t+dt. Using t here causes a one-step time
        # offset vs the MATLAB reference.
        row: Dict[str, Any] = {"time": round(t + dt, 12)}
        row.update(self.inputs)
        self.rows.append(row)

    def flush(self) -> None:
        if not self.rows:
            print(f"[Logger '{self.name}'] No data to write.")
            return
        with open(self.path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"[Logger '{self.name}'] Wrote {len(self.rows)} rows -> {self.path}")


# ──────────────────────────────────────────────────────────────
# Config helpers
# ──────────────────────────────────────────────────────────────

def load_config(path: str) -> Dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def build_components(cfg: Dict[str, Any], start: float) -> Dict[str, Component]:
    comps: Dict[str, Component] = {}
    for name, spec in cfg["components"].items():
        ctype = spec["type"]
        if ctype == "csv":
            comps[name] = CSVStimulus(
                name, spec["file"], spec.get("time_col", "time"))
        elif ctype == "const":
            comps[name] = Constant(name, spec["values"])
        elif ctype == "step":
            comps[name] = StepSignal(name, spec["ports"])
        elif ctype == "sine":
            comps[name] = SineSignal(name, spec["ports"])
        elif ctype == "ramp":
            comps[name] = RampSignal(name, spec["ports"])
        elif ctype == "pulse":
            comps[name] = PulseSignal(name, spec["ports"])
        elif ctype == "fmu":
            same_time = spec.get("same_time_evaluation", False)
            if not isinstance(same_time, bool):
                raise ValueError("same_time_evaluation must be a JSON boolean")
            fmu = FMUComponent(name, spec["file"], same_time)
            fmu.instantiate(start=start, params=spec.get("parameters", {}))
            comps[name] = fmu
        elif ctype == "logger":
            comps[name] = Logger(name, spec["file"])
        else:
            raise ValueError(f"Unknown component type '{ctype}'")
    return comps


def normalize_connections(cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [
        {
            "src_comp": c["src_comp"],
            "src_port": c["src_port"],
            "dst_comp": c["dst_comp"],
            "dst_port": c["dst_port"],
            "delayed":  bool(c.get("delayed", False)),
            "direct_feedthrough": c.get("direct_feedthrough"),
            "default":  float(c.get("default", 0.0)),
        }
        for c in cfg["connections"]
    ]


# ──────────────────────────────────────────────────────────────
# Automatic loop resolving using graph algorithms
# ──────────────────────────────────────────────────────────────

class AlgebraicLoopError(RuntimeError):
    """Raised when a same-time cycle cannot be resolved safely."""


class AlgebraicLoopConvergenceError(AlgebraicLoopError):
    """Raised when fixed-point iteration does not converge."""


def _is_cyclic_component(graph: Any, nodes: set[str]) -> bool:
    return len(nodes) > 1 or any(graph.has_edge(node, node) for node in nodes)


def _connection_feedthrough(
    connection: Dict[str, Any],
    components: Dict[str, Component],
    internal_connections: List[Dict[str, Any]],
) -> Optional[bool]:
    declared = connection.get("direct_feedthrough")
    if declared is not None:
        return bool(declared)

    destination_name = connection["dst_comp"]
    destination = components[destination_name]
    checker = getattr(destination, "has_direct_feedthrough", None)
    if checker is None:
        return None

    outgoing_ports = {
        item["src_port"] for item in internal_connections
        if item["src_comp"] == destination_name
    }
    results = [
        checker(connection["dst_port"], output_port)
        for output_port in outgoing_ports
    ]
    if any(result is True for result in results):
        return True
    if results and all(result is False for result in results):
        return False
    return None


def build_execution_plan(
    components: Dict[str, Component],
    connections: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Order acyclic components and classify cycles without modifying links."""
    if not NETWORKX_AVAILABLE:
        raise RuntimeError("networkx is required for dependency analysis")

    graph = nx.DiGraph()
    graph.add_nodes_from(components)
    for connection in connections:
        if not connection["delayed"]:
            graph.add_edge(connection["src_comp"], connection["dst_comp"])

    condensation = nx.condensation(graph)
    plan: List[Dict[str, Any]] = []
    for condensed_node in nx.topological_sort(condensation):
        names = set(condensation.nodes[condensed_node]["members"])
        if not _is_cyclic_component(graph, names):
            plan.append({"kind": "component", "components": sorted(names)})
            continue

        internal = [
            connection for connection in connections
            if not connection["delayed"]
            and connection["src_comp"] in names
            and connection["dst_comp"] in names
        ]
        feedthrough = [
            _connection_feedthrough(connection, components, internal)
            for connection in internal
        ]
        if any(value is None for value in feedthrough):
            raise AlgebraicLoopError(
                "Cannot classify cycle " + ", ".join(sorted(names))
                + ": direct-feedthrough information is missing. Add valid FMI "
                  "ModelStructure dependencies or declare 'direct_feedthrough'."
            )
        if not all(feedthrough):
            raise AlgebraicLoopError(
                "Cycle " + ", ".join(sorted(names))
                + " contains a time-advancing dependency and requires rollback. "
                  "Only pure direct-feedthrough loops are currently supported."
            )
        if any(
            isinstance(components[name], FMUComponent)
            and not components[name].same_time_evaluation
            for name in names
        ):
            raise AlgebraicLoopError(
                "Cycle " + ", ".join(sorted(names))
                + " requires verified same-time FMU output evaluation; "
                  "rollback is not implemented in this version."
            )
        plan.append({"kind": "algebraic_loop", "components": sorted(names)})

    return plan


def solve_algebraic_loop(
    component_names: List[str],
    components: Dict[str, Component],
    connections: List[Dict[str, Any]],
    *,
    absolute_tolerance: float = 1e-9,
    relative_tolerance: float = 1e-7,
    max_iterations: int = 50,
) -> int:
    """Solve a pure direct-feedthrough loop without advancing simulation time."""
    names = set(component_names)
    loop_connections = [
        connection for connection in connections
        if not connection["delayed"]
        and connection["src_comp"] in names
        and connection["dst_comp"] in names
    ]
    previous = {
        (connection["src_comp"], connection["src_port"]): float(
            components[connection["src_comp"]].get_output(connection["src_port"])
        )
        for connection in loop_connections
    }

    for iteration in range(1, max_iterations + 1):
        # Jacobi iteration uses values from the previous complete sweep.
        for connection in loop_connections:
            key = (connection["src_comp"], connection["src_port"])
            components[connection["dst_comp"]].set_input(
                connection["dst_port"], previous[key]
            )

        for name in component_names:
            components[name].evaluate_outputs()

        current = {
            key: float(components[key[0]].get_output(key[1]))
            for key in previous
        }
        if any(not math.isfinite(value) for value in current.values()):
            raise AlgebraicLoopConvergenceError(
                f"Algebraic loop produced a non-finite value on iteration {iteration}"
            )

        converged = all(
            abs(current[key] - previous[key])
            <= absolute_tolerance
            + relative_tolerance * max(abs(current[key]), abs(previous[key]))
            for key in current
        )
        previous = current
        if converged:
            for connection in loop_connections:
                key = (connection["src_comp"], connection["src_port"])
                components[connection["dst_comp"]].set_input(
                    connection["dst_port"], previous[key]
                )
            return iteration

    raise AlgebraicLoopConvergenceError(
        "Algebraic loop did not converge after "
        f"{max_iterations} iterations: {', '.join(sorted(names))}"
    )


def seed_delayed_values(
    components: Dict[str, Component],
    connections: List[Dict[str, Any]],
    user_initials: Optional[Dict[str, float]] = None,
) -> Dict[Tuple[str, str], float]:
    """Seed explicit delayed links without advancing or iterating any FMU."""
    user_initials = user_initials or {}
    delayed_state: Dict[Tuple[str, str], float] = {}

    for connection in connections:
        if not connection["delayed"]:
            continue
        key = (connection["src_comp"], connection["src_port"])
        dotted = f"{key[0]}.{key[1]}"
        if dotted in user_initials:
            value = float(user_initials[dotted])
            source = "user configuration"
        else:
            source_component = components[key[0]]
            starts = getattr(source_component, "start_values", {})
            if key[1] in starts:
                value = float(starts[key[1]])
                source = "FMU start value"
            else:
                value = float(connection["default"])
                source = "connection default"
        delayed_state[key] = value
        print(f"[delayed-init] {dotted} = {value} ({source})")

    return delayed_state


# ──────────────────────────────────────────────────────────────
# Master loop
# ──────────────────────────────────────────────────────────────

def run_case(cfg: Dict[str, Any]) -> None:
    start = float(cfg.get("start", 0.0))
    stop  = float(cfg["stop"])
    dt    = float(cfg["dt"])
    if not all(math.isfinite(value) for value in (start, stop, dt)) or dt <= 0 or stop < start:
        raise ValueError("Simulation times must be finite with dt > 0 and stop >= start")
    n_steps = (stop - start) / dt
    if not math.isclose(n_steps, round(n_steps), abs_tol=1e-9, rel_tol=0):
        raise ValueError("stop - start must be an integer multiple of dt")

    components  = build_components(cfg, start)
    connections = normalize_connections(cfg)

    automatic_plan = build_execution_plan(components, connections)

    # A manual order remains valid for an acyclic legacy experiment. It cannot
    # override an algebraic loop because no linear order can solve that loop.
    if "step_order" in cfg:
        step_order = cfg["step_order"]
        if any(stage["kind"] == "algebraic_loop" for stage in automatic_plan):
            execution_plan = automatic_plan
            step_order = [
                name
                for stage in execution_plan
                for name in stage["components"]
            ]
            print("[order] Manual step_order cannot resolve an algebraic loop; "
                  "using the automatic execution plan.")
        else:
            if len(step_order) != len(components) or set(step_order) != set(components):
                raise ValueError(
                    "Manual step_order must contain every component exactly once"
                )
            positions = {name: index for index, name in enumerate(step_order)}
            for connection in connections:
                if (not connection["delayed"]
                        and positions[connection["src_comp"]]
                        >= positions[connection["dst_comp"]]):
                    raise ValueError(
                        "Manual step_order violates a same-time connection: "
                        f"{connection['src_comp']} -> {connection['dst_comp']}"
                    )
            execution_plan = [
                {"kind": "component", "components": [name]}
                for name in step_order
            ]
            print(f"[order] Using manual step_order from JSON: {step_order}")
    else:
        execution_plan = automatic_plan
        step_order = [
            name
            for stage in execution_plan
            for name in stage["components"]
        ]
        print(f"[order] Execution plan: {execution_plan}")

    # Configure sub-stepping for every FMU and print diagnostics
    print("=== Components ===")
    for name, comp in components.items():
        if isinstance(comp, FMUComponent):
            comp.configure_stepping(dt)
            ports = comp.list_ports()
            print(f"  {name}: FMU")
            print(f"    inputs     : {ports['inputs']}")
            print(f"    outputs    : {ports['outputs']}")
            print(f"    parameters : {ports['parameters']}")
            print(f"    declared_step   = {comp.declared_step} s")
            print(f"    sub-steps/comm  = {comp._n_sub}  "
                  f"(doStep h={comp._h} s, repeated {comp._n_sub}x per dt={dt} s)")
        else:
            print(f"  {name}: {type(comp).__name__}")

    # Seed explicitly delayed links; this is not general co-initialization.
    user_initials = cfg.get("initial_values", {})
    delayed_state: Dict[Tuple[str, str], Any] = seed_delayed_values(
        components, connections, user_initials
    )

    print(f"\n=== Simulating  t=[{start}, {stop}]  dt={dt} ===")

    for step in range(round(n_steps)):
        t = round(start + step * dt, 12)
        for stage in execution_plan:
            stage_names = set(stage["components"])

            # Wire values entering this stage. Internal algebraic-loop values
            # are handled by the fixed-point solver below.
            for c in connections:
                if c["dst_comp"] not in stage_names:
                    continue
                if (stage["kind"] == "algebraic_loop"
                        and c["src_comp"] in stage_names
                        and not c["delayed"]):
                    continue
                key = (c["src_comp"], c["src_port"])
                val = (
                    delayed_state.get(key, c["default"])
                    if c["delayed"]
                    else components[c["src_comp"]].get_output(c["src_port"])
                )
                components[c["dst_comp"]].set_input(c["dst_port"], val)

            if stage["kind"] == "algebraic_loop":
                loop_cfg = cfg.get("algebraic_loop", {})
                iterations = solve_algebraic_loop(
                    stage["components"],
                    components,
                    connections,
                    absolute_tolerance=float(
                        loop_cfg.get("absolute_tolerance", 1e-9)
                    ),
                    relative_tolerance=float(
                        loop_cfg.get("relative_tolerance", 1e-7)
                    ),
                    max_iterations=int(loop_cfg.get("max_iterations", 50)),
                )
                print(
                    f"[algebraic-loop] t={t}: {stage['components']} "
                    f"converged in {iterations} iteration(s)"
                )

            # Every component advances exactly once after any same-time loop
            # has converged. Fixed-point iteration itself never calls doStep.
            for comp_name in stage["components"]:
                components[comp_name].do_step(t, dt)

        # Latch delayed outputs
        for c in connections:
            if c["delayed"]:
                key = (c["src_comp"], c["src_port"])
                delayed_state[key] = \
                    components[c["src_comp"]].get_output(c["src_port"])

    # Flush loggers and terminate FMUs
    for comp in components.values():
        if isinstance(comp, Logger):
            comp.flush()
    for comp in components.values():
        comp.terminate()

    print("=== DONE ===")


# ──────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────

def _cast(value: str) -> Any:
    """Try int → float → str."""
    try:
        return int(value)
    except (ValueError, TypeError):
        pass
    try:
        return float(value)
    except (ValueError, TypeError):
        pass
    return value


# ──────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python master_algorithm.py <case.json>")
        sys.exit(1)
    run_case(load_config(sys.argv[1]))

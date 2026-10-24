"""
generic.py  —  FMI 2.0 co-simulation master algorithm
Usage: python generic.py <case.json>

Key feature: automatic sub-stepping for fixed-step MATLAB/Simulink FMUs.
The FMU's compiled step size is read from DefaultExperiment.stepSize and used
as the internal step. The master loop's dt must be a multiple of that step.
"""

import csv
import json
import sys
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

    def __init__(self, name: str, fmu_path: str) -> None:
        if not FMPY_AVAILABLE:
            raise ImportError("Install fmpy:  pip install fmpy")
        super().__init__(name)
        self.fmu_path = fmu_path
        self.fmu: Any = None
        self.vr:              Dict[str, int] = {}
        self.var_types:       Dict[str, str] = {}
        self.inputs_list:     List[str] = []
        self.outputs_list:    List[str] = []
        self.parameters_list: List[str] = []
        self.start_values:    Dict[str, float] = {}  # initial values from FMU XML
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
        print(f"[Logger '{self.name}'] Wrote {len(self.rows)} rows → {self.path}")


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
            fmu = FMUComponent(name, spec["file"])
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
            "default":  float(c.get("default", 0.0)),
        }
        for c in cfg["connections"]
    ]


# ──────────────────────────────────────────────────────────────
# Automatic loop resolving using graph algorithms
# ──────────────────────────────────────────────────────────────

def auto_resolve_order(
    component_names: List[str],
    connections: List[Dict[str, Any]],
) -> Tuple[List[str], List[Dict[str, Any]]]:
    """
    Compute correct execution order automatically from the connection graph.

    Algorithm:
      1. Build a directed graph from non-delayed connections.
         (Delayed connections do not constrain order — they use previous-step
          values so the destination does not need the source to run first.)
      2. If the graph has no cycles -> topological sort gives the order.
      3. If cycles exist (algebraic loops):
            - find Strongly Connected Components (SCCs)
            - within each cyclic SCC pick one edge to mark as delayed
              (naive loop breaking — supervisor's "delay block" approach)
            - rebuild the graph without those edges
            - topological sort the resulting DAG

    Returns:
      step_order            : list of component names in execution order
      updated_connections   : connection list with auto-added delayed flags
    """
    if not NETWORKX_AVAILABLE:
        raise RuntimeError(
            "networkx is required for auto_resolve_order. "
            "Install it with: pip install networkx"
        )

    # Make a working copy so we do not modify the caller's list
    conns = [dict(c) for c in connections]

    # Step 1: build directed graph from non-delayed connections
    G = nx.DiGraph()
    for name in component_names:
        G.add_node(name)
    for c in conns:
        if not c["delayed"]:
            G.add_edge(c["src_comp"], c["dst_comp"])

    # Step 2: if it is already a DAG just topologically sort
    if nx.is_directed_acyclic_graph(G):
        order = list(nx.topological_sort(G))
        print(f"[auto-resolve] Graph is a DAG. Order: {order}")
        return order, conns

    # Step 3: cycles found — find SCCs and break them
    print("[auto-resolve] Algebraic loops detected. Breaking with delayed edges.")
    sccs = [scc for scc in nx.strongly_connected_components(G) if len(scc) > 1]

    for scc in sccs:
        print(f"[auto-resolve] Cycle SCC: {sorted(scc)}")
        # Pick one edge inside this SCC to delay
        subG = G.subgraph(scc)
        edge_to_delay = next(iter(subG.edges()))
        src, dst = edge_to_delay
        print(f"[auto-resolve]   marking {src} -> {dst} as delayed")

        # Mark this connection as delayed in our connection list
        for c in conns:
            if (c["src_comp"] == src
                    and c["dst_comp"] == dst
                    and not c["delayed"]):
                c["delayed"] = True
                break

        # Remove the edge from graph so the DAG order works
        G.remove_edge(src, dst)

    order = list(nx.topological_sort(G))
    print(f"[auto-resolve] Order after breaking loops: {order}")
    return order, conns


# ──────────────────────────────────────────────────────────────
# Proper initialization for feedback connections (fixed-point iteration)
# ──────────────────────────────────────────────────────────────

def compute_initial_values(
    components: Dict[str, Any],
    connections: List[Dict[str, Any]],
    step_order: List[str],
    user_initials: Optional[Dict[str, float]] = None,
    max_iters: int = 50,
    tol: float = 1e-9,
) -> Dict[Tuple[str, str], float]:
    """
    Compute consistent initial values for delayed (feedback) connections
    using fixed-point iteration at t = 0.

    Why this is needed
    ------------------
    Delayed connections require an initial value at t = 0 because their
    source FMU has not run yet. Using 0.0 as default produces wrong
    results for systems with integrators -- a non-zero true initial
    state cannot recover from a wrong initial input.

    Algorithm (CBD / fixed-point iteration)
    ---------------------------------------
    1. Seed delayed-edge values with user-specified initials,
       or 0.0 as fallback.
    2. Run one full Jacobi sweep through all components at t = 0
       using these seeded values.
    3. Read new output values produced by that sweep.
    4. If new values differ from seeded values by more than `tol`,
       update the seed and repeat from step 2.
    5. Stop when |new - old| < tol for ALL delayed edges, or
       max_iters reached (algebraic inconsistency).

    The fixed point is the set of initial values that satisfies the
    coupled equations of all FMUs simultaneously -- exactly what
    physical systems do at steady state.

    Parameters
    ----------
    components    : the component dictionary
    connections   : list of normalized connections (with delayed flags)
    step_order    : execution order from auto_resolve_order
    user_initials : optional override values from JSON
                    keys are 'comp.port' strings
    max_iters     : safety limit on iteration count
    tol           : convergence tolerance

    Returns
    -------
    delayed_state : {(src_comp, src_port): initial_value}
                    Use to seed the master loop's delayed_state.
    """
    user_initials = user_initials or {}

    # Collect every delayed edge that needs an initial value
    delayed_keys = [
        (c["src_comp"], c["src_port"])
        for c in connections
        if c["delayed"]
    ]

    if not delayed_keys:
        # No feedback -- nothing to initialize
        return {}

    print(f"[init] Computing initial values for {len(delayed_keys)} "
          f"delayed edge(s) via fixed-point iteration...")

    # Step 1: seed values for delayed edges
    # Priority order:
    #   1. user_initials from JSON      (user explicit override)
    #   2. start value from FMU XML     (set by MATLAB Integrator init)
    #   3. connection's "default" field (manual JSON fallback)
    #   4. 0.0                          (last resort)
    delayed_state: Dict[Tuple[str, str], float] = {}
    for key in delayed_keys:
        src_comp, src_port = key
        dotted = f"{src_comp}.{src_port}"

        if dotted in user_initials:
            # Priority 1: user specified in JSON
            delayed_state[key] = float(user_initials[dotted])
            print(f"[init]   seed {dotted} = {delayed_state[key]} (user JSON)")
            continue

        # Priority 2: read from FMU XML
        src_obj = components.get(src_comp)
        if hasattr(src_obj, "start_values") and src_port in src_obj.start_values:
            delayed_state[key] = src_obj.start_values[src_port]
            print(f"[init]   seed {dotted} = {delayed_state[key]} "
                  f"(from FMU XML)")
            continue

        # Priority 3: connection's default field
        for c in connections:
            if (c["src_comp"], c["src_port"]) == key and c["delayed"]:
                delayed_state[key] = float(c["default"])
                break
        print(f"[init]   seed {dotted} = "
              f"{delayed_state.get(key, 0.0)} (JSON default)")

    # Step 2-5: iterate until convergence
    t0 = 0.0       # initialization is always at start time
    init_dt = 0.0  # zero step -- evaluate equations only, no state advance

    for iteration in range(1, max_iters + 1):
        # Run one Jacobi sweep at t=0 with current seeded values
        for comp_name in step_order:
            comp = components[comp_name]

            # Wire inputs
            for c in connections:
                if c["dst_comp"] != comp_name:
                    continue
                key = (c["src_comp"], c["src_port"])
                if c["delayed"]:
                    val = delayed_state.get(key, c["default"])
                else:
                    val = components[c["src_comp"]].get_output(c["src_port"])
                comp.set_input(c["dst_port"], val)

            # Evaluate component at t=0 with zero step
            # FMUComponent.do_step with dt=0 returns current outputs without
            # advancing state. CSVStimulus reads row 0.
            try:
                comp.do_step(t0, init_dt)
            except Exception:
                # Some FMUs reject dt=0; fall back to evaluating outputs only
                pass

        # Collect new values produced by the sweep
        new_values: Dict[Tuple[str, str], float] = {}
        for key in delayed_keys:
            src_comp, src_port = key
            new_values[key] = float(
                components[src_comp].get_output(src_port)
            )

        # Check convergence
        max_diff = max(
            abs(new_values[k] - delayed_state[k]) for k in delayed_keys
        )

        # Update seed for next iteration
        delayed_state = new_values

        if max_diff < tol:
            print(f"[init] Converged in {iteration} iteration(s) "
                  f"(max diff = {max_diff:.2e}).")
            for key, val in delayed_state.items():
                print(f"[init]   {key[0]}.{key[1]} = {val}")
            return delayed_state

    # No convergence -- system has algebraic inconsistency
    print(f"[init] WARNING: did not converge in {max_iters} iterations. "
          f"Last max diff = {max_diff:.2e}. Using last values.")
    for key, val in delayed_state.items():
        print(f"[init]   {key[0]}.{key[1]} = {val}")
    return delayed_state


# ──────────────────────────────────────────────────────────────
# Master loop
# ──────────────────────────────────────────────────────────────

def run_case(cfg: Dict[str, Any]) -> None:
    start = float(cfg.get("start", 0.0))
    stop  = float(cfg["stop"])
    dt    = float(cfg["dt"])

    components  = build_components(cfg, start)
    connections = normalize_connections(cfg)

    # If step_order is provided in JSON, use it (backward compatible).
    # Otherwise compute it automatically from the connection graph.
    if "step_order" in cfg:
        step_order = cfg["step_order"]
        print(f"[order] Using manual step_order from JSON: {step_order}")
    else:
        step_order, connections = auto_resolve_order(
            list(components.keys()), connections
        )

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

    # Compute proper initial values for delayed connections using
    # fixed-point iteration. Falls back to defaults if no delayed edges.
    # User can override via "initial_values" in JSON: {"fmu1.Outport": 5.0}
    user_initials = cfg.get("initial_values", {})
    delayed_state: Dict[Tuple[str, str], Any] = compute_initial_values(
        components, connections, step_order, user_initials
    )

    print(f"\n=== Simulating  t=[{start}, {stop}]  dt={dt} ===")

    t = start
    while t <= stop + 1e-12:
        for comp_name in step_order:
            comp = components[comp_name]

            # Wire inputs from upstream outputs
            for c in connections:
                if c["dst_comp"] != comp_name:
                    continue
                key = (c["src_comp"], c["src_port"])
                val = (
                    delayed_state.get(key, c["default"])
                    if c["delayed"]
                    else components[c["src_comp"]].get_output(c["src_port"])
                )
                comp.set_input(c["dst_port"], val)

            comp.do_step(t, dt)

        # Latch delayed outputs
        for c in connections:
            if c["delayed"]:
                key = (c["src_comp"], c["src_port"])
                delayed_state[key] = \
                    components[c["src_comp"]].get_output(c["src_port"])

        t = round(t + dt, 12)

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
        print("Usage: python generic.py <case.json>")
        sys.exit(1)
    run_case(load_config(sys.argv[1]))

# FMI co-simulation master algorithm

This repository is a trial workspace for developing a Python master algorithm
that connects and runs FMI Co-Simulation FMUs. It builds on `generic.py` from
[Ci-FMI](https://github.com/mfaseehdher/Ci-FMI), my earlier thesis repository.

The [stage-0 branch](https://github.com/mfaseehdher/ci-fmi-trial/tree/stage-0)
records that starting algorithm. The `algebricloop` stage detects same-time
loops instead of automatically delaying a connection. It can iterate a loop
when component outputs update without advancing time. When FMUs support state
restoration, the master can retry a communication step. This resolves the
convergent MATLAB adder loops tested here, whose outputs update only after
`doStep`.

The `port-level-graph` stage plans the order from the step operation graph of
Hansen et al. (Definition 4): one node per `set`, `doStep` and `get`, with
edges for connections, reactive and delayed inputs, and direct feedthrough
between an input and an output. Its strongly connected components are the
algebraic loops. A loop that closes through `set` and `get` alone is a
feedthrough loop. A loop that also needs a `doStep`, such as a controller and
an integrating plant, is a reactivity loop and is solved by FMU state
rollback instead of being rejected.

The `loop-initialization` stage gives every connected input a consistent value
before the first step, following Hansen et al. (2021): an initialization
graph of ports (Definition 8), visited in topological order, with
fixed-point iteration for algebraic loops (Algorithm 1). The first guesses
come from `initial_values`, then the FMU start values, not from 0. FMUs stay
in initialization mode until their inputs are set. MATLAB-exported FMUs
compute their outputs only when they leave initialization mode, so each
iteration restores the state saved in initialization mode and leaves it
again. A loop that does not converge stops the run. This matters for MATLAB
FMUs because their first step uses the inputs given during initialization.

The master algorithm is in [`src/ci_fmi/`](src/ci_fmi/). Python tests are in
[`tests/`](tests/). MATLAB models, case configurations, and individual results
are in [the experiment folders](docs/experiments/matlab_adder_master_algorithm/).

## Requirements

- Python 3.13 with `fmpy` and `networkx`.
- For the MATLAB FMU experiments: Windows, MATLAB with Simulink and
  `exportToFMU2CS`, and a configured MinGW-w64 C compiler.

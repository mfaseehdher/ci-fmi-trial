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

The master algorithm is in [`src/ci_fmi/`](src/ci_fmi/). Python tests are in
[`tests/`](tests/). MATLAB models, case configurations, and individual results
are in [the experiment folders](docs/experiments/matlab_adder_master_algorithm/).

## Requirements

- Python 3.13 with `fmpy` and `networkx`.
- For the MATLAB FMU experiments: Windows, MATLAB with Simulink and
  `exportToFMU2CS`, and a configured MinGW-w64 C compiler.

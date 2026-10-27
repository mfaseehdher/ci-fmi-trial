# FMI co-simulation master algorithm

This repository is a trial workspace for developing a Python master algorithm
that connects and runs FMI Co-Simulation FMUs. It builds on `generic.py` from
[Ci-FMI](https://github.com/mfaseehdher/Ci-FMI), my earlier thesis repository.

The [stage-0 branch](https://github.com/mfaseehdher/ci-fmi-trial/tree/stage-0)
records that starting algorithm. The `algebricloop` stage detects same-time
loops instead of automatically delaying a connection. It can iterate a loop
when component outputs update without advancing time; it does not yet solve
same-time loops with the MATLAB FMUs used here.

The master algorithm is in [`src/ci_fmi/`](src/ci_fmi/). Python tests are in
[`tests/`](tests/). MATLAB models, case configurations, and individual results
are in [the experiment folders](docs/experiments/matlab_adder_master_algorithm/).

## Requirements

- Python 3.13 with `fmpy` and `networkx`.
- For the MATLAB FMU experiments: Windows, MATLAB with Simulink and
  `exportToFMU2CS`, and a configured MinGW-w64 C compiler.

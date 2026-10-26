# FMI co-simulation master algorithm

This GitHub trial continues the master algorithm from
[Ci-FMI](https://github.com/mfaseehdher/Ci-FMI), the repository I presented at
my thesis defense. The `stage-0` branch preserves a starting version of its
algorithm at `src/ci_fmi/master_algorithm.py`. It comes from
[Ci-FMI's initial `generic.py`](https://github.com/mfaseehdher/Ci-FMI/blob/ea27bce21f/generic.py);
the filename was changed here so each stage can be compared at the same path.

## What this stage changes

The starting algorithm automatically delayed a connection when it found a
cycle. This branch, `algebricloop`, instead distinguishes an intentional
previous-step delay from a same-time algebraic loop. It groups components in
a loop and tries fixed-point iteration only when their current outputs can be
evaluated without advancing time. It does not restore FMU state or repeatedly
call `doStep` during iteration.

The [MATLAB adder experiments](docs/experiments/matlab_adder_master_algorithm/README.md)
are documented in separate folders. The 12 configured runs produced seven
numerical passes and five deliberate rejections; a separate probe measured
when an FMU updates its output. Four rejected cases contain same-time loops
that these MATLAB Co-Simulation FMUs cannot evaluate without `doStep`; the
fifth uses an incompatible step size. The Python unit tests exercise the
fixed-point solver separately. A successful workflow does not mean that
Stage 1 solves algebraic loops between these MATLAB FMUs.

The [GitHub Actions workflow](https://github.com/mfaseehdher/ci-fmi-trial/actions)
exports the FMUs and reference traces on the MATLAB runner, then validates
the cases on a Windows Python runner. FMU state restoration is a proposed
next stage on a separate `rollback` branch; it is not part of this branch.

This trial is separate from the [university GitLab repository](https://cosysgit.uantwerpen.be/BAMA/ci-fmi),
where the weekly thesis reports belong. ChatGPT Codex assisted with this
stage's implementation, experiments, CI setup, and documentation. The
student remains responsible for checking and explaining the results.

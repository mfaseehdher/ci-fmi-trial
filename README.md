# FMI co-simulation master algorithm

This repository is a trial for continuing my MSc thesis work. The project I
presented at my thesis defense is [Ci-FMI](https://github.com/mfaseehdher/Ci-FMI).
The starting algorithm on this `stage-0` branch comes from
[that repository's `generic.py` at its initial commit](https://github.com/mfaseehdher/Ci-FMI/blob/ea27bce21f/generic.py).
I placed it at `src/ci_fmi/master_algorithm.py` so the later stages can be
compared at one path. Apart from the filename and printed usage text, this
stage does not change that algorithm. It is not a copy of every later change
on Ci-FMI's `main` branch.

## Starting behavior

The algorithm reads a JSON configuration, steps connected FMI 2.0
Co-Simulation FMUs, and derives an execution order. When it finds a cycle, it
automatically marks one connection as delayed. That permits a simulation to
run, but it changes a same-time dependency into previous-step feedback; it
does not solve an algebraic loop at the current time.

This branch has no new experiments. The `algebricloop` branch changes the
same module to detect same-time loops and adds MATLAB-FMU experiments. FMU
state restoration is reserved for a later `rollback` stage. The branches
remain available so each change can be reviewed separately.


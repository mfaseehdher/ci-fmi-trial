# Simple pendulum model

## Goal
Check whether a single exported nonlinear pendulum FMU reproduces the original Simulink angle trace.

## Setup
`pendulumSIM.slx` has two integrators. The original `setup_simple_pendulum.m` supplies their initial conditions, exposes the nonlinear angle as an Outport, and writes a MATLAB reference trace.

## How to run
Select `simple_pendulum` in the [defense-model GitHub workflow](../../../.github/workflows/defense-model-trials.yml). MATLAB exports the FMU; Python runs the master and compares the angle trace with MATLAB.

## Result
See `result.json` and the traces in the workflow artifact. This is a whole-model FMU test, not a two-FMU coupling test.

The model and setup script come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare this trial.

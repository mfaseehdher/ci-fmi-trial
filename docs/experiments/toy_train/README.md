# Toy train model

## Goal
Check whether a single exported train FMU reproduces the original Simulink train motion.

## Setup
`train.slx` has two masses and a spring. The original `setup_toy_train.m` sets its parameters, exposes engine velocity as an Outport, and writes a MATLAB reference trace.

## How to run
Select `toy_train` in the [defense-model GitHub workflow](../../../.github/workflows/defense-model-trials.yml). MATLAB exports the FMU; Python runs it with the master and compares engine velocity with the reference.

## Result
See `result.json` and the traces in the workflow artifact. This whole-model FMU does not test communication between separate FMUs.

The model and setup script come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare this trial.

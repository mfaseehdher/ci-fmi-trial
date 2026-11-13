# RC circuit: controller and plant

## Goal
Check a two-FMU controller–plant simulation for a first-order electrical system.

## Setup
`RC_Controller.slx` and `RC_Circuit_Plant.slx` become separate FMUs. `RC_Circuit_ClosedLoop_Reference.slx` gives the MATLAB reference. The plant-to-controller feedback is a same-time connection with no delay, in both the master configuration and the Simulink reference, so the master must solve the loop by FMU state rollback.

## How to run
Select `rc_circuit_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB builds the FMUs and reference; Python runs the master and compares their traces. Download the result artifact to inspect the numbers.

## Result
`result.json` reports `rollback_verified` only if every FMU repeats a step identically after restore, the master planned controller and plant as one rollback loop, every step converged with at least one retry, and disabling `setFMUstate` broke or changed the run. `metrics.csv` compares the Python master's output, computed with rollback, with the MATLAB simulation of the same closed loop. See the [experiments overview](../README.md) for these checks.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

# RC circuit: controller and plant

## Goal
Check a two-FMU controller–plant simulation for a first-order electrical system.

## Setup
`RC_Controller.slx` and `RC_Circuit_Plant.slx` become separate FMUs. `RC_Circuit_ClosedLoop_Reference.slx` gives the MATLAB reference. The plant voltage is fed back with an explicit one-step delay.

## How to run
Select `rc_circuit_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB builds the FMUs and reference; Python runs the master and compares their traces. Download the result artifact to inspect the numbers.

## Result
`result.json` records the difference between the master output and MATLAB reference. The runner removes the old manual execution order; the delay itself remains intentional.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

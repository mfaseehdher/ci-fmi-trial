# Mass–spring–damper: controller and plant

## Goal
Check two-FMU coupling for a second-order mechanical plant.

## Setup
`MassSpringDamper_Controller.slx` and `MassSpringDamper_Plant.slx` become separate FMUs. `MassSpringDamper_ClosedLoop_Reference.slx` supplies the MATLAB reference. The plant-to-controller feedback is a same-time connection with no delay, in both the master configuration and the Simulink reference, so the master must solve the loop by FMU state rollback.

## How to run
Select `mass_spring_damper_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports and simulates; Python runs the master and compares displacement. See `result.json` in the artifact.

## Result
`result.json` reports `rollback_verified` only if every FMU repeats a step identically after restore, the master planned controller and plant as one rollback loop, every step converged with at least one retry, and disabling `setFMUstate` broke or changed the run. `metrics.csv` compares the Python master's output, computed with rollback, with the MATLAB simulation of the same closed loop. See the [experiments overview](../README.md) for these checks.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

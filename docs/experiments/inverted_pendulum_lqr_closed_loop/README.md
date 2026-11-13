# Inverted pendulum: controller and plant

## Goal
Check whether the master couples a four-state cart–pendulum plant to its state-feedback controller and reproduces the reference motion.

## Setup
`InvertedPendulum_LQR_Controller.slx` and `InvertedPendulum_Plant.slx` become separate FMUs. `InvertedPendulum_ClosedLoop_Reference.slx` gives the MATLAB reference. The plant-to-controller feedback is a same-time connection with no delay, in both the master configuration and the Simulink reference, so the master must solve the loop by FMU state rollback. Both cart position and pendulum angle are compared.

## How to run
Select `inverted_pendulum_lqr_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports and simulates; Python couples the FMUs and compares both output traces. Inspect `result.json` in the artifact.

## Result
`result.json` reports `rollback_verified` only if every FMU repeats a step identically after restore, the master planned controller and plant as one rollback loop, every step converged with at least one retry, and disabling `setFMUstate` broke or changed the run. `metrics.csv` compares the Python master's output, computed with rollback, with the MATLAB simulation of the same closed loop. See the [experiments overview](../README.md) for these checks.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

# Nonlinear pendulum: controller and plant

## Goal
Check two-FMU coupling when the plant has nonlinear pendulum motion.

## Setup
`Pendulum_Controller.slx` and `Pendulum_Plant.slx` become separate FMUs. `Pendulum_ClosedLoop_Reference.slx` supplies the MATLAB reference. Feedback is explicitly delayed one communication step.

## How to run
Select `pendulum_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB builds the FMUs and reference; Python runs the master and compares the angle trace. The artifact contains `result.json`.

## Result
The recorded error describes agreement with the reference. This does not test same-time algebraic-loop resolution.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

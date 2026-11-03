# Motor speed: controller and plant

## Goal
Check whether the master can exchange speed and voltage between two FMUs and reproduce the original closed-loop motor simulation.

## Setup
`Motor_Lag_Controller.slx` and `Motor_Model.slx` are exported as separate FMUs. `Motor_Model_lag.slx` supplies the MATLAB reference. The speed feedback is deliberately delayed by one communication step in the generated configuration.

## How to run
Select `motor_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports the FMUs and reference; Python runs the master and records the difference. The generated files are in the workflow artifacts.

## Result
See `result.json` in the workflow artifact for sample counts and error metrics. This case tests intentional delayed feedback, not an algebraic-loop solver.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

# Stiff RLC circuit: controller and plant

## Goal
Check two-FMU coupling for an electrical plant with fast and slow dynamics.

## Setup
`Stiff_RLC_Controller.slx` and `Stiff_RLC_Plant.slx` become separate FMUs. `Stiff_RLC_ClosedLoop_Reference.slx` supplies the MATLAB reference. Feedback is deliberately delayed one communication step.

## How to run
Select `stiff_rlc_circuit_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB builds the FMUs and reference; Python runs and compares the circuit voltage. Inspect `result.json` in the artifact.

## Result
The error metrics describe agreement with the reference. They do not establish an undelayed-loop result.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

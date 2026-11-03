# Thermal tank: controller and plant

## Goal
Check that the master couples a controller FMU with a first-order temperature model.

## Setup
`ThermalTank_Controller.slx` and `ThermalTank_Plant.slx` become separate FMUs. `ThermalTank_ClosedLoop_Reference.slx` gives the MATLAB reference. The tank-temperature feedback is deliberately one step old.

## How to run
Select `thermal_tank_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports the FMUs and reference; Python runs and compares them. Inspect `result.json` in the artifact.

## Result
The artifact records trace differences, not an automatic accuracy pass. This is a delayed-feedback coupling test.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

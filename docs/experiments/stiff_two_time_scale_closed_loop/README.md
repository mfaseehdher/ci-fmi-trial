# Two-time-scale plant: controller and plant

## Goal
Check coupling of a controller with a plant that has a fast actuator and a slower process.

## Setup
`TwoTimeScale_Controller.slx` and `TwoTimeScale_Plant.slx` become separate FMUs. `TwoTimeScale_ClosedLoop_Reference.slx` supplies the MATLAB reference. Process feedback is intentionally delayed by one communication step.

## How to run
Select `stiff_two_time_scale_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB builds and simulates; Python runs the master and compares process output. Inspect `result.json` in the artifact.

## Result
The comparison reports measured error, not a tolerance-based pass. This is not an algebraic-loop or rollback test.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

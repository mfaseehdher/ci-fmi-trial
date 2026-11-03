# Mass–spring–damper: controller and plant

## Goal
Check two-FMU coupling for a second-order mechanical plant.

## Setup
`MassSpringDamper_Controller.slx` and `MassSpringDamper_Plant.slx` become separate FMUs. `MassSpringDamper_ClosedLoop_Reference.slx` supplies the MATLAB reference. Displacement feedback is explicitly delayed one step.

## How to run
Select `mass_spring_damper_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports and simulates; Python runs the master and compares displacement. See `result.json` in the artifact.

## Result
The artifact records the comparison error. It is a delayed-feedback coupling test, not proof of rollback.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

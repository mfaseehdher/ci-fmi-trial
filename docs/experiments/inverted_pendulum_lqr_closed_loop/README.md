# Inverted pendulum: controller and plant

## Goal
Check whether the master couples a four-state cart–pendulum plant to its state-feedback controller and reproduces the reference motion.

## Setup
`InvertedPendulum_LQR_Controller.slx` and `InvertedPendulum_Plant.slx` become separate FMUs. `InvertedPendulum_ClosedLoop_Reference.slx` gives the MATLAB reference. Plant-state feedback is intentionally delayed. Both cart position and pendulum angle are compared.

## How to run
Select `inverted_pendulum_lqr_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml). MATLAB exports and simulates; Python couples the FMUs and compares both output traces. Inspect `result.json` in the artifact.

## Result
The artifact reports separate errors for the two measured outputs. This is not an undelayed algebraic-loop test.

The setup script and models come from the earlier [Ci-FMI repository](https://github.com/mfaseehdher/Ci-FMI). ChatGPT Codex helped prepare the new trial runner and documentation.

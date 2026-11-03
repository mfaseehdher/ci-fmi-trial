# Aircraft pitch

## Goal

Run the earlier aircraft-pitch model as one FMU and compare its output with Simulink.

## Setup

The [model](aircraft_pitch_control.slx) and [original setup](setup_aircraft_pitch.m) are unchanged copies from the earlier `Ci-FMI` experiment. The [working setup](setup_working.m) follows the [earlier committed setup](https://github.com/mfaseehdher/Ci-FMI/blob/932f268744dd1d15860ce7bb1366e2eb310fc1ff/experiments/control/aircraft_pitch/setup_aircraft_pitch.m): internal input `u=0.2`, a 0.01-second fixed step for 10 seconds, and a `PitchAngle` Outport branched from the sole top-level Scope signal by [add_outports.m](../add_outports.m). The runner changes only a fresh model copy under `build/`.

## How to run

Select `aircraft_pitch` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py aircraft_pitch` from the repository root with MATLAB and its C compiler configured.

## Results

- 27 October 2026, original setup: the FMU exported, but `simOut.yout` was absent and there was no top-level output to compare.
- 31 October 2026, working setup: Simulink reference, FMU export, and master run completed. The 1,000 same-timestamp samples had RMSE `2.89e-11` and maximum absolute difference `5.00e-11` (reference CSV is rounded to 10 decimal places).

The output is the Scope signal labelled `PitchAngle` by the earlier setup. This is a whole-model self-consistency check, not a coupled-FMU test or an independent validation of the aircraft model. No acceptance tolerance has been set.

AI use: ChatGPT Codex prepared the working-copy pipeline, checked the earlier setup choices, ran the experiment, and calculated the comparison. The original model and setup remain unchanged. Link this trial from the weekly report when that report is written.

# Ball and beam

## Goal

Run the earlier ball-and-beam model as one FMU and compare its output with Simulink.

## Setup

The [model](ballBeam.slx) and [original setup](setup_ball_beam.m) are unchanged copies from the earlier `Ci-FMI` experiment. The [working setup](setup_working.m) follows the [earlier committed setup](https://github.com/mfaseehdher/Ci-FMI/blob/932f268744dd1d15860ce7bb1366e2eb310fc1ff/experiments/nonlinear/ball_beam/setup_ball_beam.m): internal input `u=0.1`, a 0.01-second fixed step for 5 seconds, and a `BallPosition` Outport branched from the sole top-level Scope signal by [add_outports.m](../add_outports.m). The runner changes only a fresh model copy under `build/`.

## How to run

Select `ball_beam` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py ball_beam` from the repository root with MATLAB and its C compiler configured.

## Results

- 27 October 2026, original setup: the FMU exported, but `simOut.yout` was absent and there was no top-level output to compare.
- 31 October 2026, working setup: Simulink reference, FMU export, and master run completed. The 500 same-timestamp samples had RMSE `2.87e-11` and maximum absolute difference `5.00e-11` (reference CSV is rounded to 10 decimal places).

This is a whole-model self-consistency check, not a coupled-FMU test. No acceptance tolerance has been set.

AI use: ChatGPT Codex prepared the working-copy pipeline, checked the earlier setup choices, ran the experiment, and calculated the comparison. The original model and setup remain unchanged. Link this trial from the weekly report when that report is written.

# Cruise control

## Goal

Run the earlier cruise-control model as one FMU and compare its velocity with Simulink.

## Setup

The [model](ccmodel.slx) and [original setup](setup_cruise_control.m) are unchanged copies from the earlier `Ci-FMI` experiment. The [working setup](setup_working.m) follows the [earlier committed setup](https://github.com/mfaseehdher/Ci-FMI/blob/932f268744dd1d15860ce7bb1366e2eb310fc1ff/experiments/first_order/cruise_control/setup_cruise_control.m): `Kp=800`, `Ki=40`, reference input `u=10`, and a 0.01-second fixed step for 120 seconds. It branches the model's sole top-level Scope signal to a `Velocity` Outport using [add_outports.m](../add_outports.m). This happens only on a fresh model copy under `build/`.

## How to run

Select `cruise_control` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py cruise_control` from the repository root with MATLAB and its C compiler configured. The runner works on a copy under `build/`.

## Results

- 27 October 2026, original setup: blocked by undefined `Ki`, `Kp`, and `u`, with no top-level Outport.
- 31 October 2026, working setup: Simulink reference, FMU export, and master run completed. The 12,000 same-timestamp velocity samples had RMSE `1.54e-11` and maximum absolute difference `5.00e-11` (reference CSV is rounded to 10 decimal places).

This checks one complete cruise-control FMU against the same Simulink model. It does not test coupling a separate controller FMU to a plant FMU, and no acceptance tolerance has been set.

AI use: ChatGPT Codex prepared the working-copy pipeline, checked the earlier setup values, ran the experiment, and calculated the comparison. The original model and setup remain unchanged. Link this trial from the weekly report when that report is written.

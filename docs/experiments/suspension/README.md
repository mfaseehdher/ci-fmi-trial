# Suspension

## Goal

Run the earlier suspension model as one FMU with a defined external input and compare its displacement with Simulink.

## Setup

The [model](suspmod.mdl) and [original setup](setup_suspension.m) are unchanged copies from the earlier `Ci-FMI` experiment. The [working setup](setup_working.m) uses the lowercase `m1`, `m2`, `k1`, and `k2` names from the [earlier committed setup](https://github.com/mfaseehdher/Ci-FMI/blob/932f268744dd1d15860ce7bb1366e2eb310fc1ff/experiments/second_order/suspension/setup_suspension.m). Unlike that script, it explicitly feeds `0.1` through the top-level `In` port for both Simulink and the master, with a 0.01-second fixed step for 5 seconds. The `Out` port is already present.

## How to run

Select `suspension` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py suspension` from the repository root with MATLAB and its C compiler configured.

## Results

- 27 October 2026, original setup: blocked by uppercase parameter names where the model uses lowercase names.
- 31 October 2026, working setup: Simulink reference, FMU export, and master run completed. The 500 same-timestamp samples had RMSE `7.18e-9` and maximum absolute difference `2.26e-8`. The reference peak is only `2.70e-7`, so the relative difference is worth examining; this is not declared a pass.

The old pipeline's reference for this model was all zero while its FMU output was nonzero. This working setup removes that input mismatch, but it still tests one complete FMU rather than coupling separate FMUs. No acceptance tolerance has been set.

AI use: ChatGPT Codex prepared the matched-input working setup, ran the experiment, and calculated the comparison. The original model and setup remain unchanged. Link this trial from the weekly report when that report is written.

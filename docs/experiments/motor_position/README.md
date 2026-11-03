# Motor position

## Goal

Check whether the master can run the motor-position FMU and reproduce the earlier Simulink position trace.

## Setup

`Motor_Pos.slx` and `setup_motor_position.m` are unchanged copies from `Ci-FMI/experiments/stiff/motor_position`. The setup simulates 10 seconds with a constant voltage of 1 and a 0.001-second fixed step. It writes the Simulink reference. The exported FMU has one input (`Voltage`) and one output (`Position`).

## How to run

Select `motor_position` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py motor_position` from the repository root with MATLAB and its C compiler configured.

## Result (27 October 2026)

MATLAB simulation and FMU export succeeded. The master ran the FMU for 10,000 steps and compared `Position` with the reference at the same timestamps. RMSE was `9.5521e-5`; maximum absolute difference was `9.9900e-5`. These are measured differences, not a declared pass against a tolerance. The FMU output is slightly behind the reference at the end of the run. This trial uses one complete motor FMU, not a coupled multi-FMU system.

At `t=10.000`, the FMU reported `0.9390209193`, matching the Simulink reference at `t=9.999`; the reference at `t=10.000` was `0.9391208194`. The same-timestamp comparison above remains unchanged.

AI use: ChatGPT Codex copied the original files, made the runner, and calculated the comparison. No reference values were adjusted. Link this trial from the weekly report when that report is written.

# Motor speed

## Goal

Check whether the master can run the motor-speed FMU and reproduce the earlier Simulink speed trace.

## Setup

`Motor_Model.slx` and `setup_motor_speed.m` are unchanged copies from `Ci-FMI/experiments/control/motor_speed`. The setup simulates 3 seconds with a constant input of 1 and a 0.001-second fixed step. It writes the Simulink reference. MATLAB exports the FMU from a working copy, and the runner discovers its one input (`In1`) and one output (`Out1`).

## How to run

Select `motor_speed` in the defense-model workflow, or run `py -3.13 docs/experiments/run_defense_models.py motor_speed` from the repository root with MATLAB and its C compiler configured. The runner creates a fresh folder under `build/` and keeps the reference and FMU output separately.

## Result (27 October 2026)

MATLAB simulation and FMU export succeeded. The master ran the FMU for 3,000 steps and compared `Out1` with the reference `Speed` at the same timestamps. RMSE was `5.2678e-5`; maximum absolute difference was `1.3373e-4`. These are measured differences, not a declared pass against a tolerance. The FMU output begins at zero at the first logged step, while the Simulink reference is already nonzero there. This trial uses one complete motor FMU, not a coupled multi-FMU system.

At `t=3.000`, the FMU reported `0.0995921476`, matching the Simulink reference at `t=2.999`; the reference at `t=3.000` was `0.0995927636`. This is evidence of a one-step output timing difference, not a reason to shift the reference when reporting the comparison.

AI use: ChatGPT Codex copied the original files, made the runner, and calculated the comparison. No reference values were adjusted. Link this trial from the weekly report when that report is written.

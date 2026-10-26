# When does an FMU update its output?

## Goal
Find out whether an input change updates this FMU's output immediately.

## Setup
[probe_same_time.py](probe_same_time.py) uses the [weighted adder FMU](../models/weighted_adder.slx). It sets the fixed input to 1 and the gain to 0.5, then changes feedback from 0 to 2. The new output should be 2.

## How to run
MATLAB makes the FMU. The probe reads its output before the input change, just after the change, and after one time step.

## Result
The readings were 0, 0, then 2. This FMU updated only after a time step. The probe saves its readings in `results/same_time_probe.json`; see the [CI run](https://github.com/mfaseehdher/ci-fmi-trial/actions/runs/40338211493). This finding is about this FMU, not every FMU.

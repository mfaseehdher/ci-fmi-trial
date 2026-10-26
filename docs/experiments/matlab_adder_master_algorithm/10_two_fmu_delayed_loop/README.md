# Delayed feedback between two FMUs

## Goal
Check that two FMUs work together when one connection uses a previous-step value.

## Setup
In [case.json](case.json), [A](../models/network_adder_a.slx) adds 1 to B's previous output, starting at 0. [B](../models/network_adder_b.slx) adds 2 to A's current output. The delay is explicitly configured.

## How to run
MATLAB makes both FMUs and a delayed-feedback reference. The master runs the FMUs. Compare B's output at each step.

## Result
Both gave 3, 6, 9, 12. The reference is a single Simulink model, not two FMUs. [Experiment 11](../11_two_fmu_same_time_loop/) uses different FMUs, so it is not a delay-only comparison.
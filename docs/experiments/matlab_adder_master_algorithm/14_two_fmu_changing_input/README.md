# Two-FMU loop with a changing input

## Goal
Check that rollback solves the same loop at several communication steps, including after the input changes.

## Setup
The same [A](../models/loop_adder_a.slx) and [B](../models/loop_adder_b.slx) FMUs used in [experiment 11](../11_two_fmu_same_time_loop/) satisfy A = r + 0.5B and B = 0.2A. In [case.json](case.json), r is 1 for the first two steps and 2 for the third. The hand-calculated answers are A = 10r/9 and B = 2r/9.

## How to run
MATLAB makes both FMUs. Run this case through the experiment runner and compare both outputs at 0.1, 0.2, and 0.3 seconds.

## Result
The recorded A values were 1.1111111, 1.1111111, and 2.2222222; B was 0.2222222, 0.2222222, and 0.4444444. All matched the analytical answers within the configured tolerance. A local rollback trace showed that each trial restarted at the beginning of its communication step before one result was accepted.

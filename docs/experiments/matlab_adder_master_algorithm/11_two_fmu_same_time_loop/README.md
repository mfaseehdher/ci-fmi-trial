# Same-time loop between two FMUs

## Goal
Check that the master solves a same-time loop between two MATLAB FMUs.

## Setup
In [case.json](case.json), [A](../models/loop_adder_a.slx) follows A = 1 + 0.5B and [B](../models/loop_adder_b.slx) follows B = 0.2A. The mathematical answers are A = 10/9 and B = 2/9. Neither connection is delayed.

## How to run
MATLAB makes both FMUs. Run this case through the experiment runner and compare both outputs with the calculated answers.

## Result
The master restored both FMUs before each trial and converged in 15 iterations. It recorded A = 1.1111111 and B = 0.2222222 at 0.1 seconds, within the configured tolerance of 10/9 and 2/9. Neither connection was delayed.

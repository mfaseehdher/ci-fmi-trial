# Same-time feedback with a possible answer

## Goal
Check whether the master can solve a loop without a delay.

## Setup
In [case.json](case.json), the [weighted adder](../models/weighted_adder.slx) follows y = 1 + 0.5y. The correct mathematical answer is 2. Unlike [experiment 05](../05_weighted_delayed_feedback/), this connection is not delayed.

## How to run
MATLAB makes the FMU. Run this case through the experiment runner and compare the output with 2.

## Result
The master restored the FMU to the start of the step between trials. It converged in 24 iterations and recorded 1.99999988 at 0.1 seconds, within the configured tolerance of the analytical answer 2.

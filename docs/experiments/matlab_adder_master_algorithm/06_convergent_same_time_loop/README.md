# Same-time feedback with a possible answer

## Goal
Check whether the master can solve a loop without a delay.

## Setup
In [case.json](case.json), the [weighted adder](../models/weighted_adder.slx) follows y = 1 + 0.5y. The correct mathematical answer is 2. Unlike [experiment 05](../05_weighted_delayed_feedback/), this connection is not delayed.

## How to run
MATLAB makes the FMU. The master tries to run the loop and reports its result.

## Result
The master detected the loop but rejected it. It did not calculate 2 because this FMU updates its output only after a time step.

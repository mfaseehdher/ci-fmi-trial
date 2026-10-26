# Same-time feedback with gain above one

## Goal
Check what happens when the loop gain is above one.

## Setup
In [case.json](case.json), the [weighted adder](../models/weighted_adder.slx) follows y = 1 + 1.2y with no delay. Its mathematical answer is -5, but repeated guesses starting at zero would move away from that answer.

## How to run
MATLAB makes the FMU. The master tries the loop and reports its result.

## Result
The master rejected the FMU before trying to solve the loop, because its output does not update without a time step. This run does not test whether the solver diverges.

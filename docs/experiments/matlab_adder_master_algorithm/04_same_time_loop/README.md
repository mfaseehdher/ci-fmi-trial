# Same-time loop in one adder

## Goal
Check that a loop with no solution is reported as nonconvergent.

## Setup
In [case.json](case.json), the [adder FMU](../models/adder.slx) receives 1 and its own current output. This gives the equation y = 1 + y, which has no finite answer.

## How to run
MATLAB makes the FMU. Run this case through the experiment runner.

## Result
The master restored the FMU state between 50 trial steps. The guesses did not converge: each one was 1 larger than the previous one. It reported a convergence error and did not add a delay or write an answer.

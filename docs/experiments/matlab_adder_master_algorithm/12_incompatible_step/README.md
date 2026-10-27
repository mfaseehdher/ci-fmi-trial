# Unsuitable time step

## Goal
Check that the master reports an FMU step-size problem.

## Setup
The [slow adder](../models/slow_adder.slx) needs steps of 0.2 seconds. [case.json](case.json) asks for 0.1-second steps. Its inputs are 1 and 2; there is no loop.

## How to run
MATLAB makes the FMU. The master tries to run it and reports whether the step is allowed.

## Result
The master rejected the 0.1-second step because it is not a multiple of 0.2 seconds. This is not an algebraic-loop test.

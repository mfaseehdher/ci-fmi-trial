# Same-time loop in one adder

## Goal
Check that the master detects a loop without adding a delay.

## Setup
In [case.json](case.json), the [adder FMU](../models/adder.slx) receives 1 and its own current output. This gives the equation y = 1 + y, which has no finite answer.

## How to run
MATLAB makes the FMU. The master tries the case and reports whether it can handle the loop.

## Result
The master detected the loop and rejected it. This rejection does not prove it recognized that the equation has no answer. it rejected because master detected that the adder’s output was connected back to its input without a delay. To solve this, it would need to try different feedback values at the same simulation time. In this experiment, immediate output updates were not verified for the FMU, and the master could not rewind the FMU after a time step to try another value. It therefore rejected the loop instead of producing an unreliable result. This test demonstrates loop detection, not loop solving.
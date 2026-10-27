# Delayed feedback with half the previous answer

## Goal
Check feedback when half of the previous answer is returned.

## Setup
The [weighted adder](../models/weighted_adder.slx) adds 1 to half its feedback input. In [case.json](case.json), feedback starts at 0 and is explicitly delayed.

## How to run
MATLAB makes the FMU and a delayed-feedback reference. The master runs the FMU. Compare the outputs.

## Result
Both gave 1, 1.5, 1.75, 1.875. The master did not add this delay.

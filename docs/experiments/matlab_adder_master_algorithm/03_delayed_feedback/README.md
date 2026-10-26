# Feedback from the previous step

## Goal
Check that an explicit delay uses the previous answer.

## Setup
[case.json](case.json) gives the [adder FMU](../models/adder.slx) a fixed input of 1. Its other input gets the previous answer, starting at 0. The delay is written in the configuration.

## How to run
MATLAB makes the FMU and a delayed-feedback reference. The master runs the FMU. Compare the outputs at each step.

## Result
Both gave 1, 2, 3, 4. The master did not add the delay itself. This is not a same-time loop test.
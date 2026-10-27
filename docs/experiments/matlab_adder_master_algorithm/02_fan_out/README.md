# One value sent to two inputs

## Goal
Check that one value can be sent to both inputs of an FMU.

## Setup
[case.json](case.json) sends 2 to both inputs of the [adder model](../models/adder.slx), so the answer should be 4.

## How to run
MATLAB makes the FMU and a reference answer. The master runs the FMU. Compare the answers.

## Result
Both answers were 4 at each of three steps. This tests the connections.

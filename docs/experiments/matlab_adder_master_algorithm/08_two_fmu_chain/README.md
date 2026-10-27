# Two FMUs in a chain

## Goal
Check that the output of one FMU can be used by another.

## Setup
In [case.json](case.json), [FMU A](../models/network_adder_a.slx) adds 2 + 3 = 5. It sends 5 to [FMU B](../models/network_adder_b.slx), which adds 4.

## How to run
MATLAB makes both FMUs and a reference for the final sum. The master runs the two FMUs in order. Compare the final answers.

## Result
Both gave 9 at each of three steps. The reference is a single Simulink sum, not a separate two-FMU simulation. There is no feedback.

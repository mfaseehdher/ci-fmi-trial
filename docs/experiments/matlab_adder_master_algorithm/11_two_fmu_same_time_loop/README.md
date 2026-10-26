# Same-time loop between two FMUs

## Goal
Check that the master detects a loop where two FMUs need each other's current output.

## Setup
In [case.json](case.json), [A](../models/loop_adder_a.slx) follows A = 1 + 0.5B and [B](../models/loop_adder_b.slx) follows B = 0.2A. The mathematical answers are A = 10/9 and B = 2/9. Neither connection is delayed.

## How to run
MATLAB makes both FMUs. The master tries to run the loop and reports its result.

## Result
The master detected the loop but rejected it before stepping. The master detected that A and B need each other’s current output. It rejected the loop before stepping because same-time output evaluation was not verified for these FMUs, and this version cannot restore their state to try multiple values at the same time.

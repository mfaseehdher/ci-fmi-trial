# Two paths join at a third FMU

## Goal
Check that the master can choose the order for three connected FMUs.

## Setup
In [case.json](case.json), [A](../models/network_adder_a.slx) makes 2 + 1 = 3 and [B](../models/network_adder_b.slx) makes 2 + 3 = 5. [C](../models/network_adder_c.slx) adds their outputs. No order is entered by hand.

## How to run
MATLAB makes the three FMUs and a reference for the final sum. The master runs the connected FMUs. Compare the final answers.

## Result
Both gave 8 at each of three steps. The reference checks the final sum, not the three-FMU arrangement. There is no feedback.
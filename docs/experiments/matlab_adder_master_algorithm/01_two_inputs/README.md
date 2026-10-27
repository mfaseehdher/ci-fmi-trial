# Two inputs

## Goal
Check that the master can run one adder FMU.

## Setup
The [adder model](../models/adder.slx) adds two inputs. [case.json](case.json) gives it 2 and 3.

## How to run
MATLAB makes the FMU and runs the model directly for a reference. The master runs the FMU with the same inputs. Compare the answers.

## Result
Both answers were 5 at each of three steps. This tests basic input and output, not feedback. See the [CI run](https://github.com/mfaseehdher/ci-fmi-trial/actions/runs/40338211493).

AI use: ChatGPT Codex helped prepare this experiment.

# Experiments

Each experiment belongs in its own folder. Its README records the goal, setup, result and how to run it.
MATLAB-generated FMUs and output files are produced by the pipeline and are not
committed as evidence of a run that has not happened.

The [defense-model runner](run_defense_models.py) accepts an experiment folder
containing `case.json` and a Simulink `.slx` or `.mdl` file. Each case in this trial
keeps the original MATLAB setup script; cases that needed repairs also have a
separate `setup_working.m` selected by `case.json`. The runner copies the model, asks
MATLAB to simulate and export it, reads the FMU ports and declared step, runs
the master, and compares samples at identical timestamps when a reference
exists. A setup may provide a single external input through workspace vectors
`t` and `u`; otherwise an input stimulus must be supplied explicitly. It never
guesses one. Generated files stay in each case's ignored `build/` directory.

The [manual GitHub workflow](../../.github/workflows/defense-model-trials.yml)
does the same for one selected case or all case folders. `compared` means
metrics were calculated, not that a tolerance was met. A model with missing
parameters, no usable output, or no valid input source is reported as blocked.
The workflow uploads both successful traces and failure logs. It requires a
Windows self-hosted runner with MATLAB R2026b, a C compiler, and Python 3.13.

This pipeline tests each supplied Simulink model as one FMU. It cannot infer
how to split a whole Simulink model into separate controller and plant FMUs, or
which signals should connect those FMUs, from the `.slx` file alone. Such a
coupled experiment needs an explicit model boundary and connection mapping.

The [coupled-model workflow](../../.github/workflows/coupled-model-trials.yml)
uses eight earlier controller and plant model pairs, plus the cruise-control
model split into a PI controller and a vehicle, in separate experiment
folders. MATLAB exports both FMUs and a closed-loop reference in its first
job; Python downloads them, runs the master, and compares output traces in its
second job. These pairs are now rollback tests: the plant-to-controller
feedback is a same-time connection, and the Simulink reference has no
`Unit Delay` in that path either. Earlier versions delayed this feedback by one
step in both places, which avoided the loop instead of solving it. The runner
removes the manual `step_order` and refuses a configuration that still delays
the feedback.

The master must then treat controller and plant as one loop and solve each
communication step by rollback: save both FMU states, step, compare the
exchanged values, restore, and repeat until they agree. A case is
`rollback_verified` only when all of these hold:

1. Restore probe: each FMU is stepped, restored and stepped again with the same
   inputs; both steps give identical outputs. The probe also records whether
   the FMU has state that changes from step to step, because restoring a
   stateless FMU proves nothing.
2. The master planned `controller` and `plant` as one `rollback_step` loop.
3. Every communication step converged, and at least one needed a retry, so a
   restore actually happened during the run.
4. Running again with `setFMUstate` disabled breaks the run or changes the
   trace by more than 1e-6 of the signal range. This shows the result depends
   on the restore.

The result of the experiment is the comparison between the Python master's
output, computed with rollback, and the MATLAB simulation of the same closed
loop at matching timestamps. The disabled-restore run is only part of the
rollback check and is not compared with MATLAB. A setup script may also write
a closed-form solution as `<case>_exact.csv`; the runner then measures both
the master and the MATLAB simulation against it (`exact_solution` in
`result.json`). Only the cruise-control case has one.

Each comparison records RMSE (squared-error average), MAE (absolute-error
average), maximum absolute error, and relative RMSE as a percentage of the
MATLAB signal's range. It also records a windowed DTW mean absolute cost that
allows up to five samples of time shift. DTW is a secondary shape check, not
an accuracy verdict: the time-aligned errors remain visible. `result.json`
and `metrics.csv` are in each run's artifact. There is no default PASS/FAIL
accuracy threshold because one threshold would not suit every signal.

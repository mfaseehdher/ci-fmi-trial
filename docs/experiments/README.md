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
uses eight earlier controller and plant model pairs in separate experiment
folders. MATLAB exports both FMUs and a closed-loop reference in its first
job; Python downloads them, runs the master, and compares output traces in its
second job. The original configurations explicitly delay feedback. The runner
removes their manual `step_order` but does not remove or invent a delay. These
trials therefore check coupling and automatic ordering, not rollback or
same-time algebraic-loop resolution.

Each comparison records RMSE (squared-error average), MAE (absolute-error
average), maximum absolute error, and relative RMSE as a percentage of the
MATLAB signal's range. It also records a windowed DTW mean absolute cost that
allows up to five samples of time shift. DTW is a secondary shape check, not
an accuracy verdict: the time-aligned errors remain visible. `result.json`
and `metrics.csv` are in each run's artifact. There is no default PASS/FAIL
accuracy threshold because one threshold would not suit every signal.

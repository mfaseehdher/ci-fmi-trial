# MATLAB adder FMUs: Stage 1 validation

This experiment checks the master algorithm after adding same-time cycle
detection and an initial fixed-point solver, but before adding FMU rollback.
All integration FMUs are generated from the Simulink models in `models/`.
The Python-only tests in `tests/test_stage1.py` check the solver logic; they
are not evidence that a MATLAB FMU can be evaluated without stepping.

## Cases

| Config | What it checks | Expected in Stage 1 |
| --- | --- | --- |
| `01_two_inputs` | One adder, two constant inputs | 5 at each step |
| `02_fan_out` | One source drives two adder inputs | 4 at each step |
| `03_delayed_feedback` | Feedback explicitly uses the previous output | 1, 2, 3, 4 |
| `04_same_time_loop` | Feedback uses the current output | Reject without changing the link to delayed |
| `05_weighted_delayed_feedback` | Explicit delay with gain 0.5 | 1, 1.5, 1.75, 1.875 |
| `06_convergent_same_time_loop` | A mathematical fixed point exists | Reject: this FMU cannot be evaluated at the same time without a step |
| `07_divergent_same_time_loop` | Loop gain does not support simple fixed-point convergence | Reject for the same FMU limitation, before testing convergence |
| `08_two_fmu_chain` | Data passes from one FMU to another | 9 at each step |
| `09_branch_and_merge` | Several source paths reach one adder | 8 at each step |
| `10_two_fmu_delayed_loop` | Two FMUs with an explicit one-step delay | 3, 6, 9, 12 |
| `11_two_fmu_same_time_loop` | Two FMUs depend on one another in the current step | Reject without inserting a delay |
| `12_incompatible_step` | Requested step conflicts with the FMU's compiled step | Reject with a step-size error |

The acceptance criterion is **7 passing cases and 5 expected rejections**.
An expected rejection is not a solved algebraic loop. In particular, the
two-FMU same-time case is the one to revisit when FMU state restoration is
implemented in Stage 2. Its equations are `a = 1 + 0.5*b` and `b = 0.2*a`,
so a future successful result should be `a = 1/0.9` and `b = 0.2/0.9`.

The `probe_same_time.py` check is important: changing a MATLAB FMU input
leaves its output at 0 until `doStep` is called; then the output becomes 2.
The R2026b FMUs used locally advertise `canGetAndSetFMUstate=true`, but that
does not make their outputs immediately evaluable. Stage 1 deliberately
refuses to iterate them with `doStep` because doing so repeatedly without
restoring state would advance simulation time during a single loop solve.

## Run locally

Requirements: MATLAB R2026b with Simulink and FMU Builder support, a supported
C compiler for FMU export, and Python 3.13 with FMPy, NetworkX, and pytest.
From the repository root on Windows:

```powershell
$env:MW_MINGW64_LOC = 'PATH_TO_YOUR_MINGW64_DIRECTORY'
& "$env:ProgramFiles\MATLAB\R2026b\bin\matlab.exe" -batch "mex -setup C; addpath(fullfile(pwd,'docs','experiments','matlab_adder_master_algorithm')); generate_artifacts"
python -m pip install fmpy==0.3.26 networkx==3.6.1 pytest==8.4.2
python -m pytest tests -q
python docs/experiments/matlab_adder_master_algorithm/run_experiments.py
python docs/experiments/matlab_adder_master_algorithm/probe_same_time.py
```

`generate_artifacts.m` recreates the models and exports the FMUs. It also
simulates `adder.slx` and a separate `reference_feedback.slx` containing an
explicit Unit Delay. The seven generated `references/*.csv` files are compared
sample by sample with the master's CSVs and the expected hand-calculated
values. These references are for the passing cases only; rejected loops have
no simulated reference claim in Stage 1. The runner records each case in
`results/summary.json` and `results/summary.md`, and the probe writes
`results/same_time_probe.json`. Generated FMUs, references, and results are
not source files to commit.

## GitHub Actions

`matlab-export.yml` uses the `matlab`-labelled self-hosted Windows runner to
export the FMUs and Simulink references. It uploads them as workflow artifacts.
On this trial machine, it uses the already installed MinGW64 directory if
`MW_MINGW64_LOC` is unset; another runner must set that variable itself.
`fmu-validation.yml` downloads those artifacts on a GitHub-hosted Windows
runner, runs the tests and cases, and uploads the results. The self-hosted
job is triggered only by trusted branch pushes or manual dispatch, not pull
requests. The local validation passed; a GitHub Actions run must still be
checked after this branch is pushed.

## Traceability and AI use

This GitHub repository is a trial. The related thesis weekly update belongs
in the university [GitLab weekly-report directory](https://cosysgit.uantwerpen.be/BAMA/ci-fmi/-/tree/main/docs/weeklies),
with an explicit link back to this experiment and its CI run. This Stage 1
experiment, scripts, solver code, and CI workflow were prepared with ChatGPT
Codex assistance under Muhammad Fasih's direction. The student should review
the models, explain the 7/5 outcomes, and rerun the commands before presenting
the result as thesis evidence.

# MATLAB adder experiments

Each numbered folder is one experiment. Its README explains the goal, the connections, how it was checked, and the result. Its `case.json` contains the master's input settings.

| Experiment | Result |
| --- | --- |
| [01 Two inputs](01_two_inputs/) | 5, 5, 5 |
| [02 One value to two inputs](02_fan_out/) | 4, 4, 4 |
| [03 Delayed feedback](03_delayed_feedback/) | 1, 2, 3, 4 |
| [04 Same-time self-loop](04_same_time_loop/) | Did not converge; no solution |
| [05 Weighted delayed feedback](05_weighted_delayed_feedback/) | 1, 1.5, 1.75, 1.875 |
| [06 Solvable same-time self-loop](06_convergent_same_time_loop/) | About 2 |
| [07 Higher-gain same-time self-loop](07_divergent_same_time_loop/) | Did not converge |
| [08 Two-FMU chain](08_two_fmu_chain/) | 9, 9, 9 |
| [09 Branch and merge](09_branch_and_merge/) | 8, 8, 8 |
| [10 Two-FMU delayed feedback](10_two_fmu_delayed_loop/) | 3, 6, 9, 12 |
| [11 Two-FMU same-time loop](11_two_fmu_same_time_loop/) | A = 10/9, B = 2/9 |
| [12 Unsuitable time step](12_incompatible_step/) | Rejected |
| [13 Output timing probe](13_same_time_output_probe/) | Output changed only after a time step |
| [14 Two-FMU loop with changing input](14_two_fmu_changing_input/) | Solved at three communication steps |

## How to run them

In MATLAB, run [generate_artifacts.m](generate_artifacts.m) to make the FMUs and the reference files for the non-loop cases. Then run [run_experiments.py](run_experiments.py) with Python to check the numbered cases with `case.json`. Run [probe_same_time.py](13_same_time_output_probe/probe_same_time.py) separately for experiment 13. The [GitHub workflow](https://github.com/mfaseehdher/ci-fmi-trial/actions/workflows/matlab-export.yml) runs these steps automatically.

Of the 13 `case.json` experiments, seven match MATLAB reference traces and three same-time loops match hand-calculated answers. Two loops do not converge under fixed-point iteration; the unsuitable-step case is rejected before stepping. The separate probe shows why these MATLAB FMUs require rollback for loop trials: their outputs change only after `doStep`. Generated FMUs, references, and results are not stored in Git.

AI use: ChatGPT Codex assisted with integrating the rollback code and preparing the experiment checks. The loop answers were checked against the equations in the individual experiment READMEs.

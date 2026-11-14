# Cruise control: controller and plant

## Goal
Split the [cruise-control model](../cruise_control/README.md) into a PI controller FMU and a vehicle FMU, solve their same-time feedback loop by rollback, and compare the Python master with both the MATLAB simulation and the exact solution.

## Setup
[ccmodel.slx](ccmodel.slx) is an unchanged copy of the single-FMU case's model. It stays whole and gives the MATLAB reference; the setup script only adds a `Velocity` Outport to its Scope signal on the copy under `build/`. The [setup script](setup_cruise_control_closed_loop.m) builds two new models from the same blocks:

- `CruiseControl_PI_Controller`: `ReferenceVelocity` and `MeasuredVelocity` in, `Force = Kp*e + Ki*∫e` out, with `e = ReferenceVelocity - MeasuredVelocity`.
- `CruiseControl_Vehicle`: `Force` in, `Velocity` out, from `m*v' + b*v = Force`.

The values are the single-FMU case's: `m = 1000`, `b = 50`, `Kp = 800`, `Ki = 40`, a reference step to `u = 10` m/s at `t = 0`, and fixed-step `ode4` at 0.01 s for 120 s. The master's step source gives the same reference step. The plant-to-controller feedback is a same-time connection with no delay, in both the master configuration and the Simulink reference, so the master must solve the loop by FMU state rollback. There is no `step_order`; the master plans the loop itself.

With these values `Ki/Kp = b/m`, so the controller zero cancels the slow closed-loop pole and the velocity is exactly `v(t) = u*(1 - exp(-(Kp/m)*t)) = 10*(1 - exp(-0.8*t))`. The setup script writes this at the reference's timestamps to `cruise_control_closed_loop_exact.csv` and stops with an error if the gains no longer satisfy the cancellation.

## How to run
Select `cruise_control_closed_loop` in the [coupled-model GitHub workflow](../../../.github/workflows/coupled-model-trials.yml), or run from the repository root with MATLAB R2026b and its C compiler configured:

```bash
py -3.13 docs/experiments/run_coupled_models.py build --run-id local-cruise cruise_control_closed_loop
```

```bash
py -3.13 docs/experiments/run_coupled_models.py check --run-id local-cruise cruise_control_closed_loop
```

## Result
`result.json` reports `rollback_verified` under the same four checks as the other coupled cases; see the [experiments overview](../README.md). `metrics.csv` compares the Python master's output, computed with rollback, with the MATLAB simulation. `exact_solution` in `result.json`, `metrics_exact_master.csv` and `metrics_exact_matlab.csv` measure the master and the MATLAB simulation against the exact solution, so neither is treated as the truth.

12 November 2026, local run with MATLAB R2026b: `rollback_verified`. Both FMUs replayed a step identically after restore and both are stateful. The master planned `controller` and `plant` as one `rollback_step` loop. All 12,000 steps converged, 1,573 of them needed a retry, with at most 3 iterations. Disabling `setFMUstate` made the controller FMU reject the next step (`fmi2DoStep` status 3).

| Velocity compared | RMSE | Max absolute error | Relative RMSE |
| --- | ---: | ---: | ---: |
| Python master with rollback vs MATLAB simulation | 4.63e-3 | 7.97e-2 | 0.047 % |
| Python master with rollback vs exact solution | 4.63e-3 | 7.97e-2 | 0.047 % |
| MATLAB simulation vs exact solution | 1.99e-11 | 2.0e-10 | 2e-10 % |

The MATLAB simulation equals the exact solution to rounding, so the master's error is co-simulation error, not a reference error. In that run the master trace lagged by one 0.01 s step; the first sample was 0 instead of 0.0797. The cause was initialization, not time labels: a MATLAB-exported FMU takes its first step with the input it was given in initialization mode, and ignores the value set just before that `fmi2DoStep`. The master left the inputs at their start value 0, so the vehicle's first step used `Force = 0`. A direct check on the vehicle FMU confirms it: with `Force = 8000` set before every step, the velocity after the first step is 0 when initialization used `Force = 0`, and 0.07998 when it used `Force = 8000`.

14 November 2026, same FMUs, master with fixed-point initialization (`loop-initialization` branch): the master now solves the controller–plant loop at `t = 0` before the first step (2 iterations, `Force = 8000`, `Velocity = 0`) and gives these inputs to the FMUs in initialization mode. The result is still `rollback_verified`, with 1,572 steps needing a retry and at most 3 iterations.

| Velocity compared | RMSE | Max absolute error | Relative RMSE |
| --- | ---: | ---: | ---: |
| Python master with rollback vs MATLAB simulation | 1.93e-3 | 1.39e-2 | 0.019 % |
| Python master with rollback vs exact solution | 1.93e-3 | 1.39e-2 | 0.019 % |

The remaining error comes from the coupling: the controller's force is held constant during each step instead of following the error inside it.

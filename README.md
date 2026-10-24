# FMI master algorithm trial

This is a GitHub trial for the thesis master algorithm. It is not a copy of the
supervisor's GitLab repository. Only the paths needed for the prototype and its
experiments are mirrored, so reviewed changes can later be transferred to GitLab.

- `src/ci_fmi/`: master algorithm code and the unchanged legacy reference.
- `docs/experiments/`: one folder per MATLAB/Simulink FMU experiment, each with
  its model, scripts, configuration, results description, and AI-use record.

The planned review sequence is `stage-0` (legacy reference), `algebricloop`
(same-time loop detection and no-rollback behavior), then `rollback` (FMU state
restoration). Each branch is reviewed before the next stage starts. No experiment
result is claimed until its commands and CI job have run successfully.

Weekly reports and other thesis documentation remain in the GitLab repository.

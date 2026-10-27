function create_and_export_loop_adders()
% Export two distinct weighted adders for the solvable two-FMU loop.

experiment_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(experiment_dir, 'models');
fmu_dir = fullfile(experiment_dir, 'fmus');
if ~exist(model_dir, 'dir'), mkdir(model_dir); end
if ~exist(fmu_dir, 'dir'), mkdir(fmu_dir); end

build_one('loop_adder_a', '0.5', model_dir, fmu_dir);
build_one('loop_adder_b', '0.2', model_dir, fmu_dir);
end


function build_one(model_name, gain, model_dir, fmu_dir)
if bdIsLoaded(model_name), close_system(model_name, 0); end
new_system(model_name);
add_block('simulink/Sources/In1', [model_name '/Bias'], ...
    'Position', [40 45 70 65]);
add_block('simulink/Sources/In1', [model_name '/Feedback'], ...
    'Port', '2', 'Position', [40 105 70 125]);
add_block('simulink/Math Operations/Gain', [model_name '/LoopGain'], ...
    'Gain', gain, 'Position', [115 95 185 135]);
add_block('simulink/Math Operations/Sum', [model_name '/Add'], ...
    'Inputs', '++', 'Position', [235 55 265 115]);
add_block('simulink/Sinks/Out1', [model_name '/Out'], ...
    'Position', [325 75 355 95]);

add_line(model_name, 'Bias/1', 'Add/1');
add_line(model_name, 'Feedback/1', 'LoopGain/1');
add_line(model_name, 'LoopGain/1', 'Add/2');
add_line(model_name, 'Add/1', 'Out/1');

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.1', ...
    'StartTime', '0.0', ...
    'StopTime', '10.0');
save_system(model_name, fullfile(model_dir, [model_name '.slx']));
exportToFMU2CS(model_name, 'SaveDirectory', fmu_dir);
close_system(model_name, 0);
fprintf('Created and exported %s with loop gain %s\n', model_name, gain);
end

function create_and_export_network_adders()
% Create separately named MATLAB FMUs for multi-component graph tests.

experiment_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(experiment_dir, 'models');
fmu_dir = fullfile(experiment_dir, 'fmus');
if ~exist(model_dir, 'dir'), mkdir(model_dir); end
if ~exist(fmu_dir, 'dir'), mkdir(fmu_dir); end

specs = {
    'network_adder_a', '0.1';
    'network_adder_b', '0.1';
    'network_adder_c', '0.1';
    'slow_adder',      '0.2';
};

for index = 1:size(specs, 1)
    build_and_export(specs{index, 1}, specs{index, 2}, model_dir, fmu_dir);
end
end


function build_and_export(model_name, fixed_step, model_dir, fmu_dir)
if bdIsLoaded(model_name), close_system(model_name, 0); end
new_system(model_name);

add_block('simulink/Sources/In1', [model_name '/In1'], ...
    'Position', [40 45 70 65]);
add_block('simulink/Sources/In1', [model_name '/In2'], ...
    'Port', '2', 'Position', [40 105 70 125]);
add_block('simulink/Math Operations/Sum', [model_name '/Add'], ...
    'Inputs', '++', 'Position', [150 55 180 115]);
add_block('simulink/Sinks/Out1', [model_name '/Out'], ...
    'Position', [250 75 280 95]);
add_line(model_name, 'In1/1', 'Add/1');
add_line(model_name, 'In2/1', 'Add/2');
add_line(model_name, 'Add/1', 'Out/1');

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', fixed_step, ...
    'StartTime', '0.0', ...
    'StopTime', '10.0');
save_system(model_name, fullfile(model_dir, [model_name '.slx']));
exportToFMU2CS(model_name, 'SaveDirectory', fmu_dir);
close_system(model_name, 0);
fprintf('Created and exported %s (step %s s)\n', model_name, fixed_step);
end

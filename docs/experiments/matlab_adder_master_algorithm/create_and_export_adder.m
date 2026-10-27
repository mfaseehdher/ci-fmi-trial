function create_and_export_adder()
% Recreate the two-input adder in Simulink and export it as FMI 2.0 CS.

experiment_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(experiment_dir, 'models');
fmu_dir = fullfile(experiment_dir, 'fmus');
model_name = 'adder';

if ~exist(model_dir, 'dir'), mkdir(model_dir); end
if ~exist(fmu_dir, 'dir'), mkdir(fmu_dir); end
if bdIsLoaded(model_name), close_system(model_name, 0); end

new_system(model_name);
add_block('simulink/Sources/In1', [model_name '/Inport'], ...
    'Position', [40 45 70 65]);
add_block('simulink/Sources/In1', [model_name '/Inport1'], ...
    'Port', '2', 'Position', [40 105 70 125]);
add_block('simulink/Math Operations/Sum', [model_name '/Add'], ...
    'Inputs', '++', 'Position', [150 55 180 115]);
add_block('simulink/Sinks/Out1', [model_name '/Outport'], ...
    'Position', [250 75 280 95]);

add_line(model_name, 'Inport/1', 'Add/1');
add_line(model_name, 'Inport1/1', 'Add/2');
add_line(model_name, 'Add/1', 'Outport/1');

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.1', ...
    'StartTime', '0.0', ...
    'StopTime', '10.0');

save_system(model_name, fullfile(model_dir, [model_name '.slx']));
exportToFMU2CS(model_name, 'SaveDirectory', fmu_dir);
close_system(model_name, 0);

fprintf('Created %s and exported %s\n', ...
    fullfile(model_dir, [model_name '.slx']), ...
    fullfile(fmu_dir, [model_name '.fmu']));
end

function create_and_export_weighted_adder()
% Create a tunable weighted adder and export it as FMI 2.0 Co-Simulation.
%
% Equation: Out = Bias + FeedbackGain * Feedback

experiment_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(experiment_dir, 'models');
fmu_dir = fullfile(experiment_dir, 'fmus');
model_name = 'weighted_adder';

if ~exist(model_dir, 'dir'), mkdir(model_dir); end
if ~exist(fmu_dir, 'dir'), mkdir(fmu_dir); end
if bdIsLoaded(model_name), close_system(model_name, 0); end

feedback_gain = Simulink.Parameter(0.5);
feedback_gain.CoderInfo.StorageClass = 'ExportedGlobal';
assignin('base', 'feedback_gain', feedback_gain);

new_system(model_name);
add_block('simulink/Sources/In1', [model_name '/Bias'], ...
    'Position', [40 45 70 65]);
add_block('simulink/Sources/In1', [model_name '/Feedback'], ...
    'Port', '2', 'Position', [40 105 70 125]);
add_block('simulink/Math Operations/Gain', [model_name '/FeedbackGain'], ...
    'Gain', 'feedback_gain', 'Position', [115 95 185 135]);
add_block('simulink/Math Operations/Sum', [model_name '/Add'], ...
    'Inputs', '++', 'Position', [235 55 265 115]);
add_block('simulink/Sinks/Out1', [model_name '/Out'], ...
    'Position', [325 75 355 95]);

add_line(model_name, 'Bias/1', 'Add/1');
add_line(model_name, 'Feedback/1', 'FeedbackGain/1');
add_line(model_name, 'FeedbackGain/1', 'Add/2');
add_line(model_name, 'Add/1', 'Out/1');

set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.1', ...
    'StartTime', '0.0', ...
    'StopTime', '10.0');

save_system(model_name, fullfile(model_dir, [model_name '.slx']));
exportToFMU2CS(model_name, ...
    'SaveDirectory', fmu_dir, ...
    'ExportedParameters', {'feedback_gain'}, ...
    'ExportedParameterNames', {'FeedbackGain'});
close_system(model_name, 0);

fprintf('Created %s and exported %s\n', ...
    fullfile(model_dir, [model_name '.slx']), ...
    fullfile(fmu_dir, [model_name '.fmu']));
end

% setup_motor_closed_loop.m
% Coupled two-FMU validation case for the DC motor speed model.
%
% Reference:
%   Motor_Model_lag.slx runs the monolithic closed-loop Simulink model.
%
% Co-simulation under test:
%   Motor_Lag_Controller.fmu -> Motor_Model.fmu with plant speed fed back to
%   the controller. generic.py executes this coupled system from JSON.

testDir = pwd;
t_stop = 3;
dt = 0.001;

% CTMS DC motor plant parameters.
K = 0.01;
J = 0.01;
b = 0.1;
R = 1.0;
L = 0.5;
assignin('base', 'K', K);
assignin('base', 'J', J);
assignin('base', 'b', b);
assignin('base', 'R', R);
assignin('base', 'L', L);

plantModel = 'Motor_Model';
controllerModel = 'Motor_Lag_Controller';
referenceModel = 'Motor_Model_lag';

configure_model_for_export(plantModel, t_stop, dt);
set_param(plantModel, 'LoadExternalInput', 'off');
save_system(plantModel);
fprintf('  Exporting plant FMU: %s\n', plantModel);
exportToFMU2CS(plantModel, 'SaveDirectory', testDir);
close_system(plantModel, 0);

configure_model_for_export(controllerModel, t_stop, dt);
save_system(controllerModel);
fprintf('  Exporting controller FMU: %s\n', controllerModel);
exportToFMU2CS(controllerModel, 'SaveDirectory', testDir);
close_system(controllerModel, 0);

configure_model_for_export(referenceModel, t_stop, dt);
save_system(referenceModel);
fprintf('  Running MATLAB closed-loop reference: %s\n', referenceModel);
simOut = sim(referenceModel, 'ReturnWorkspaceOutputs', 'on');

t_out = simOut.tout(:);
raw = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    speed = raw{1}.Values.Data(:);
else
    speed = raw(:, 1);
end

n = min(numel(t_out), numel(speed));
ref_csv = fullfile(testDir, 'motor_closed_loop_ref.csv');
fid = fopen(ref_csv, 'w');
fprintf(fid, 'time,Speed\r\n');
for i = 1:n
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), speed(i));
end
fclose(fid);
close_system(referenceModel, 0);
fprintf('  Reference saved: motor_closed_loop_ref.csv (%d rows)\n', n);

write_coupled_json(testDir, t_stop, dt);
fprintf('  Coupled JSON saved: motor_closed_loop.json\n');


function configure_model_for_export(model, t_stop, dt)
if bdIsLoaded(model)
    close_system(model, 0);
end
load_system(model);
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'ode4');
set_param(model, 'FixedStep', num2str(dt));
set_param(model, 'StopTime', num2str(t_stop));
set_param(model, 'SaveState', 'off');
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveFormat', 'Dataset');
end


function write_coupled_json(testDir, t_stop, dt)
components = struct();
components.reference = struct( ...
    'type', 'step', ...
    'ports', struct( ...
        'ReferenceSpeed', struct( ...
            'initial', 0.0, ...
            'step', 1.0, ...
            'step_time', 0.0)));
components.controller = struct( ...
    'type', 'fmu', ...
    'file', 'Motor_Lag_Controller.fmu');
components.plant = struct( ...
    'type', 'fmu', ...
    'file', 'Motor_Model.fmu');
components.log = struct( ...
    'type', 'logger', ...
    'file', 'results_motor_closed_loop.csv');

connections = {
    conn('reference', 'ReferenceSpeed', 'controller', 'ReferenceSpeed')
    conn('plant', 'Out1', 'controller', 'MeasuredSpeed', true, 0.0)
    conn('controller', 'Voltage', 'plant', 'In1')
    conn('plant', 'Out1', 'log', 'Speed')
};

experiment = struct();
experiment.start = 0.0;
experiment.stop = t_stop;
experiment.dt = dt;
experiment.components = components;
experiment.connections = connections;
experiment.step_order = {'reference', 'controller', 'plant', 'log'};

json_str = jsonencode(experiment, 'PrettyPrint', true);
json_path = fullfile(testDir, 'motor_closed_loop.json');
fid = fopen(json_path, 'w');
fprintf(fid, '%s', json_str);
fclose(fid);
end


function c = conn(src_comp, src_port, dst_comp, dst_port, delayed, default)
if nargin < 5
    delayed = false;
end
if nargin < 6
    default = 0.0;
end
c = struct( ...
    'src_comp', src_comp, ...
    'src_port', src_port, ...
    'dst_comp', dst_comp, ...
    'dst_port', dst_port, ...
    'delayed', delayed, ...
    'default', default);
end

% setup_cruise_control_closed_loop.m
% Coupled two-FMU rollback case for the cruise-control PI loop.
%
% Reference:
%   ccmodel.slx is the unchanged cruise-control model from the cruise_control
%   case. Its sole Scope signal is exposed as the Velocity Outport on this
%   working copy and simulated as the monolithic MATLAB reference.
%
% Co-simulation under test:
%   CruiseControl_PI_Controller.fmu -> CruiseControl_Vehicle.fmu, with the
%   vehicle velocity fed back to the controller at the same time.
%
% Exact solution:
%   m*v' + b*v = F and F = Kp*e + Ki*int(e), e = r - v. When Ki/Kp = b/m the
%   controller zero cancels the slow closed-loop pole and, for a step r = u
%   at t = 0, v(t) = u*(1 - exp(-(Kp/m)*t)).

testDir = pwd;
caseName = 'cruise_control_closed_loop';
t_stop = 120;
dt = 0.01;

m = 1000;
b = 50;
Kp = 800;
Ki = 40;
u = 10;
assignin('base', 'm', m);
assignin('base', 'b', b);
assignin('base', 'Kp', Kp);
assignin('base', 'Ki', Ki);
assignin('base', 'u', u);

plantModel = 'CruiseControl_Vehicle';
controllerModel = 'CruiseControl_PI_Controller';
referenceModel = 'ccmodel';

create_vehicle_plant(plantModel, m, b, t_stop, dt);
export_model(plantModel, testDir);

create_pi_controller(controllerModel, Kp, Ki, t_stop, dt);
export_model(controllerModel, testDir);

% add_outports.m lives in docs/experiments; this script runs in
% docs/experiments/<case>/build/<run-id>.
addpath(fullfile(testDir, '..', '..', '..'));
load_system(referenceModel);
add_outports(referenceModel, 'Velocity');
configure_model(referenceModel, t_stop, dt);
set_param(referenceModel, 'LoadExternalInput', 'off');
save_system(referenceModel);
t_ref = run_reference(referenceModel, caseName, 'Velocity');

write_exact_csv(sprintf('%s_exact.csv', caseName), t_ref, m, b, Kp, Ki, u);

write_coupled_json(testDir, caseName, t_stop, dt, u, ...
    'CruiseControl_PI_Controller.fmu', 'CruiseControl_Vehicle.fmu', ...
    'ReferenceVelocity', 'MeasuredVelocity', 'Force', 'Velocity');


function create_vehicle_plant(model, m, b, t_stop, dt)
% Same blocks as the "plant model" subsystem of ccmodel.slx.
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/Force'], 'Position', [40 80 70 100]);
add_block('simulink/Math Operations/Sum', [model '/Sum'], ...
    'Inputs', '+-', 'Position', [120 75 145 125]);
add_block('simulink/Math Operations/Gain', [model '/Inertia'], ...
    'Gain', sprintf('1/%g', m), 'Position', [190 85 250 115]);
add_block('simulink/Continuous/Integrator', [model '/Integrator'], ...
    'InitialCondition', '0', 'Position', [300 80 330 120]);
add_block('simulink/Math Operations/Gain', [model '/damping'], ...
    'Gain', num2str(b), 'Orientation', 'left', 'Position', [190 160 250 190]);
add_block('simulink/Sinks/Out1', [model '/Velocity'], 'Position', [410 90 440 110]);
add_line(model, 'Force/1', 'Sum/1', 'autorouting', 'on');
add_line(model, 'Sum/1', 'Inertia/1', 'autorouting', 'on');
add_line(model, 'Inertia/1', 'Integrator/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'Velocity/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'damping/1', 'autorouting', 'on');
add_line(model, 'damping/1', 'Sum/2', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_pi_controller(model, Kp, Ki, t_stop, dt)
% Same blocks as the controller part of ccmodel.slx.
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/ReferenceVelocity'], 'Position', [40 50 70 70]);
add_block('simulink/Sources/In1', [model '/MeasuredVelocity'], 'Position', [40 130 70 150]);
add_block('simulink/Math Operations/Sum', [model '/Error'], ...
    'Inputs', '+-', 'Position', [130 75 155 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [230 40 290 70]);
add_block('simulink/Continuous/Integrator', [model '/Integrator'], ...
    'InitialCondition', '0', 'Position', [210 130 240 170]);
add_block('simulink/Math Operations/Gain', [model '/Ki'], ...
    'Gain', num2str(Ki), 'Position', [280 135 340 165]);
add_block('simulink/Math Operations/Sum', [model '/Sum'], ...
    'Inputs', '++', 'Position', [390 80 415 130]);
add_block('simulink/Sinks/Out1', [model '/Force'], 'Position', [470 95 500 115]);
add_line(model, 'ReferenceVelocity/1', 'Error/1', 'autorouting', 'on');
add_line(model, 'MeasuredVelocity/1', 'Error/2', 'autorouting', 'on');
add_line(model, 'Error/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'Error/1', 'Integrator/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'Ki/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'Sum/1', 'autorouting', 'on');
add_line(model, 'Ki/1', 'Sum/2', 'autorouting', 'on');
add_line(model, 'Sum/1', 'Force/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function write_exact_csv(fileName, t, m, b, Kp, Ki, u)
if abs(Ki / Kp - b / m) > 1e-12
    error('The closed-form solution needs Ki/Kp = b/m (got %g and %g).', Ki / Kp, b / m);
end
v = u * (1 - exp(-(Kp / m) * t));
fid = fopen(fileName, 'w');
fprintf(fid, 'time,Velocity\r\n');
for i = 1:numel(t)
    fprintf(fid, '%.10f,%.10f\r\n', t(i), v(i));
end
fclose(fid);
fprintf('  Exact solution saved: %s (%d rows)\n', fileName, numel(t));
end


function export_model(model, testDir)
fprintf('  Exporting FMU: %s\n', model);
exportToFMU2CS(model, 'SaveDirectory', testDir);
close_system(model, 0);
end


function t_out = run_reference(model, caseName, signalName)
fprintf('  Running MATLAB closed-loop reference: %s\n', model);
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
t_out = save_reference_csv(simOut, sprintf('%s_ref.csv', caseName), signalName);
close_system(model, 0);
end


function t_out = save_reference_csv(simOut, fileName, signalName)
t_out = simOut.tout(:);
raw = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    y = raw{1}.Values.Data(:);
else
    y = raw(:, 1);
end
n = min(numel(t_out), numel(y));
t_out = t_out(1:n);
fid = fopen(fileName, 'w');
fprintf(fid, 'time,%s\r\n', signalName);
for i = 1:n
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), y(i));
end
fclose(fid);
fprintf('  Reference saved: %s (%d rows)\n', fileName, n);
end


function configure_model(model, t_stop, dt)
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'ode4');
set_param(model, 'FixedStep', num2str(dt));
set_param(model, 'StopTime', num2str(t_stop));
set_param(model, 'SaveState', 'off');
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveFormat', 'Dataset');
end


function reset_model(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
if isfile([model '.slx'])
    delete([model '.slx']);
end
end


function write_coupled_json(testDir, caseName, t_stop, dt, stepValue, controllerFmu, plantFmu, refPort, measPort, controlPort, outputPort)
components.reference = struct('type', 'step', 'ports', struct(refPort, struct('initial', 0.0, 'step', stepValue, 'step_time', 0.0)));
components.controller = struct('type', 'fmu', 'file', controllerFmu);
components.plant = struct('type', 'fmu', 'file', plantFmu);
components.log = struct('type', 'logger', 'file', sprintf('results_%s.csv', caseName));
connections = {
    conn('reference', refPort, 'controller', refPort)
    conn('plant', outputPort, 'controller', measPort)
    conn('controller', controlPort, 'plant', controlPort)
    conn('plant', outputPort, 'log', outputPort)
};
experiment.start = 0.0;
experiment.stop = t_stop;
experiment.dt = dt;
experiment.components = components;
experiment.connections = connections;
json_str = jsonencode(experiment, 'PrettyPrint', true);
fid = fopen(fullfile(testDir, sprintf('%s.json', caseName)), 'w');
fprintf(fid, '%s', json_str);
fclose(fid);
fprintf('  Coupled JSON saved: %s.json\n', caseName);
end


function c = conn(src_comp, src_port, dst_comp, dst_port, delayed, default)
if nargin < 5, delayed = false; end
if nargin < 6, default = 0.0; end
c = struct('src_comp', src_comp, 'src_port', src_port, ...
           'dst_comp', dst_comp, 'dst_port', dst_port, ...
           'delayed', delayed, 'default', default);
end

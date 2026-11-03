% setup_pendulum_closed_loop.m
% Coupled two-FMU validation case for a nonlinear pendulum plant.

testDir = pwd;
caseName = 'pendulum_closed_loop';
t_stop = 5;
dt = 0.001;

plantModel = 'Pendulum_Plant';
controllerModel = 'Pendulum_Controller';
referenceModel = 'Pendulum_ClosedLoop_Reference';

g_over_l = 9.81;
damping = 0.4;
Kp = 18.0;
Kd = 4.5;

create_pendulum_plant(plantModel, g_over_l, damping, t_stop, dt);
export_model(plantModel, testDir);

create_pd_controller(controllerModel, Kp, Kd, t_stop, dt);
export_model(controllerModel, testDir);

create_pendulum_reference(referenceModel, g_over_l, damping, Kp, Kd, t_stop, dt);
run_reference(referenceModel, caseName, 'Angle');

write_coupled_json(testDir, caseName, t_stop, dt);


function create_pendulum_plant(model, g_over_l, damping, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/Torque'], 'Position', [35 100 65 120]);
add_block('simulink/Math Operations/Sum', [model '/AccelerationSum'], ...
    'Inputs', '+--', 'Position', [160 95 185 145]);
add_block('simulink/Continuous/Integrator', [model '/Omega'], ...
    'InitialCondition', '0', 'Position', [250 95 280 125]);
add_block('simulink/Continuous/Integrator', [model '/Theta'], ...
    'InitialCondition', '0', 'Position', [350 95 380 125]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin_theta'], ...
    'Operator', 'sin', 'Position', [430 165 465 195]);
add_block('simulink/Math Operations/Gain', [model '/Gravity'], ...
    'Gain', num2str(g_over_l), 'Position', [500 165 560 195]);
add_block('simulink/Math Operations/Gain', [model '/Damping'], ...
    'Gain', num2str(damping), 'Position', [350 220 410 250]);
add_block('simulink/Sinks/Out1', [model '/Angle'], 'Position', [500 90 530 110]);
add_block('simulink/Sinks/Out1', [model '/AngularVelocity'], 'Position', [500 125 530 145]);
add_line(model, 'Torque/1', 'AccelerationSum/1', 'autorouting', 'on');
add_line(model, 'AccelerationSum/1', 'Omega/1', 'autorouting', 'on');
add_line(model, 'Omega/1', 'Theta/1', 'autorouting', 'on');
add_line(model, 'Theta/1', 'Angle/1', 'autorouting', 'on');
add_line(model, 'Omega/1', 'AngularVelocity/1', 'autorouting', 'on');
add_line(model, 'Theta/1', 'sin_theta/1', 'autorouting', 'on');
add_line(model, 'sin_theta/1', 'Gravity/1', 'autorouting', 'on');
add_line(model, 'Gravity/1', 'AccelerationSum/2', 'autorouting', 'on');
add_line(model, 'Omega/1', 'Damping/1', 'autorouting', 'on');
add_line(model, 'Damping/1', 'AccelerationSum/3', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_pd_controller(model, Kp, Kd, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/ReferenceAngle'], 'Position', [35 50 65 70]);
add_block('simulink/Sources/In1', [model '/Angle'], 'Position', [35 115 65 135]);
add_block('simulink/Sources/In1', [model '/AngularVelocity'], 'Position', [35 205 65 225]);
add_block('simulink/Math Operations/Sum', [model '/AngleError'], ...
    'Inputs', '+-', 'Position', [140 75 165 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [220 85 280 115]);
add_block('simulink/Math Operations/Gain', [model '/Kd'], ...
    'Gain', num2str(Kd), 'Position', [220 195 280 225]);
add_block('simulink/Math Operations/Sum', [model '/TorqueSum'], ...
    'Inputs', '+-', 'Position', [340 115 365 165]);
add_block('simulink/Sinks/Out1', [model '/Torque'], 'Position', [430 130 460 150]);
add_line(model, 'ReferenceAngle/1', 'AngleError/1', 'autorouting', 'on');
add_line(model, 'Angle/1', 'AngleError/2', 'autorouting', 'on');
add_line(model, 'AngleError/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'AngularVelocity/1', 'Kd/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'TorqueSum/1', 'autorouting', 'on');
add_line(model, 'Kd/1', 'TorqueSum/2', 'autorouting', 'on');
add_line(model, 'TorqueSum/1', 'Torque/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_pendulum_reference(model, g_over_l, damping, Kp, Kd, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/Step', [model '/ReferenceStep'], ...
    'Time', '0', 'Before', '0', 'After', '0.4', 'Position', [30 60 65 90]);
add_block('simulink/Discrete/Unit Delay', [model '/AngleDelay'], ...
    'InitialCondition', '0', 'SampleTime', num2str(dt), ...
    'Position', [130 140 180 180]);
add_block('simulink/Discrete/Unit Delay', [model '/VelocityDelay'], ...
    'InitialCondition', '0', 'SampleTime', num2str(dt), ...
    'Position', [130 230 180 270]);
add_block('simulink/Math Operations/Sum', [model '/AngleError'], ...
    'Inputs', '+-', 'Position', [230 75 255 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [310 85 370 115]);
add_block('simulink/Math Operations/Gain', [model '/Kd'], ...
    'Gain', num2str(Kd), 'Position', [310 220 370 250]);
add_block('simulink/Math Operations/Sum', [model '/TorqueSum'], ...
    'Inputs', '+-', 'Position', [430 130 455 180]);
add_block('simulink/Math Operations/Sum', [model '/AccelerationSum'], ...
    'Inputs', '+--', 'Position', [520 130 545 180]);
add_block('simulink/Continuous/Integrator', [model '/Omega'], ...
    'InitialCondition', '0', 'Position', [610 130 640 160]);
add_block('simulink/Continuous/Integrator', [model '/Theta'], ...
    'InitialCondition', '0', 'Position', [710 130 740 160]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin_theta'], ...
    'Operator', 'sin', 'Position', [780 220 815 250]);
add_block('simulink/Math Operations/Gain', [model '/Gravity'], ...
    'Gain', num2str(g_over_l), 'Position', [850 220 910 250]);
add_block('simulink/Math Operations/Gain', [model '/Damping'], ...
    'Gain', num2str(damping), 'Position', [710 275 770 305]);
add_block('simulink/Sinks/Out1', [model '/Angle'], 'Position', [830 135 860 155]);
add_line(model, 'ReferenceStep/1', 'AngleError/1', 'autorouting', 'on');
add_line(model, 'AngleDelay/1', 'AngleError/2', 'autorouting', 'on');
add_line(model, 'AngleError/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'VelocityDelay/1', 'Kd/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'TorqueSum/1', 'autorouting', 'on');
add_line(model, 'Kd/1', 'TorqueSum/2', 'autorouting', 'on');
add_line(model, 'TorqueSum/1', 'AccelerationSum/1', 'autorouting', 'on');
add_line(model, 'AccelerationSum/1', 'Omega/1', 'autorouting', 'on');
add_line(model, 'Omega/1', 'Theta/1', 'autorouting', 'on');
add_line(model, 'Theta/1', 'Angle/1', 'autorouting', 'on');
add_line(model, 'Theta/1', 'AngleDelay/1', 'autorouting', 'on');
add_line(model, 'Omega/1', 'VelocityDelay/1', 'autorouting', 'on');
add_line(model, 'Theta/1', 'sin_theta/1', 'autorouting', 'on');
add_line(model, 'sin_theta/1', 'Gravity/1', 'autorouting', 'on');
add_line(model, 'Gravity/1', 'AccelerationSum/2', 'autorouting', 'on');
add_line(model, 'Omega/1', 'Damping/1', 'autorouting', 'on');
add_line(model, 'Damping/1', 'AccelerationSum/3', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function export_model(model, testDir)
fprintf('  Exporting FMU: %s\n', model);
exportToFMU2CS(model, 'SaveDirectory', testDir);
close_system(model, 0);
end


function run_reference(model, caseName, signalName)
fprintf('  Running MATLAB closed-loop reference: %s\n', model);
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
save_reference_csv(simOut, sprintf('%s_ref.csv', caseName), signalName);
close_system(model, 0);
end


function save_reference_csv(simOut, fileName, signalName)
t_out = simOut.tout(:);
raw = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    y = raw{1}.Values.Data(:);
else
    y = raw(:, 1);
end
n = min(numel(t_out), numel(y));
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


function write_coupled_json(testDir, caseName, t_stop, dt)
components.reference = struct('type', 'step', 'ports', struct('ReferenceAngle', struct('initial', 0.0, 'step', 0.4, 'step_time', 0.0)));
components.controller = struct('type', 'fmu', 'file', 'Pendulum_Controller.fmu');
components.plant = struct('type', 'fmu', 'file', 'Pendulum_Plant.fmu');
components.log = struct('type', 'logger', 'file', sprintf('results_%s.csv', caseName));
connections = {
    conn('reference', 'ReferenceAngle', 'controller', 'ReferenceAngle')
    conn('plant', 'Angle', 'controller', 'Angle', true, 0.0)
    conn('plant', 'AngularVelocity', 'controller', 'AngularVelocity', true, 0.0)
    conn('controller', 'Torque', 'plant', 'Torque')
    conn('plant', 'Angle', 'log', 'Angle')
};
experiment.start = 0.0;
experiment.stop = t_stop;
experiment.dt = dt;
experiment.components = components;
experiment.connections = connections;
experiment.step_order = {'reference', 'controller', 'plant', 'log'};
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

% setup_mass_spring_damper_closed_loop.m
% Coupled two-FMU validation case for a mass-spring-damper plant.

testDir = pwd;
caseName = 'mass_spring_damper_closed_loop';
t_stop = 5;
dt = 0.001;

plantModel = 'MassSpringDamper_Plant';
controllerModel = 'MassSpringDamper_Controller';
referenceModel = 'MassSpringDamper_ClosedLoop_Reference';

m = 1.0;
c = 0.8;
k = 4.0;
Kp = 10.0;

create_second_order_plant(plantModel, m, c, k, t_stop, dt);
export_model(plantModel, testDir);

create_p_controller(controllerModel, Kp, t_stop, dt);
export_model(controllerModel, testDir);

create_second_order_reference(referenceModel, m, c, k, Kp, t_stop, dt);
run_reference(referenceModel, caseName, 'Displacement');

write_coupled_json(testDir, caseName, t_stop, dt, ...
    'MassSpringDamper_Controller.fmu', 'MassSpringDamper_Plant.fmu', ...
    'ReferenceDisplacement', 'MeasuredDisplacement', 'Force', 'Displacement');


function create_second_order_plant(model, m, c, k, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/Force'], 'Position', [40 80 70 100]);
add_block('simulink/Continuous/Transfer Fcn', [model '/MassSpringDamper'], ...
    'Numerator', sprintf('[%g]', 1 / m), ...
    'Denominator', sprintf('[1 %g %g]', c / m, k / m), ...
    'Position', [150 70 280 110]);
add_block('simulink/Sinks/Out1', [model '/Displacement'], 'Position', [360 80 390 100]);
add_line(model, 'Force/1', 'MassSpringDamper/1', 'autorouting', 'on');
add_line(model, 'MassSpringDamper/1', 'Displacement/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_p_controller(model, Kp, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/ReferenceDisplacement'], 'Position', [40 50 70 70]);
add_block('simulink/Sources/In1', [model '/MeasuredDisplacement'], 'Position', [40 130 70 150]);
add_block('simulink/Math Operations/Sum', [model '/Error'], ...
    'Inputs', '+-', 'Position', [140 75 165 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [230 85 290 115]);
add_block('simulink/Sinks/Out1', [model '/Force'], 'Position', [360 90 390 110]);
add_line(model, 'ReferenceDisplacement/1', 'Error/1', 'autorouting', 'on');
add_line(model, 'MeasuredDisplacement/1', 'Error/2', 'autorouting', 'on');
add_line(model, 'Error/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'Force/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_second_order_reference(model, m, c, k, Kp, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/Step', [model '/ReferenceDisplacementStep'], ...
    'Time', '0', 'Before', '0', 'After', '1', 'Position', [30 70 65 100]);
add_block('simulink/Discrete/Unit Delay', [model '/FeedbackDelay'], ...
    'InitialCondition', '0', 'SampleTime', num2str(dt), ...
    'Position', [185 155 235 195]);
add_block('simulink/Math Operations/Sum', [model '/Error'], ...
    'Inputs', '+-', 'Position', [130 75 155 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [215 85 275 115]);
add_block('simulink/Continuous/Transfer Fcn', [model '/MassSpringDamper'], ...
    'Numerator', sprintf('[%g]', 1 / m), ...
    'Denominator', sprintf('[1 %g %g]', c / m, k / m), ...
    'Position', [340 75 470 115]);
add_block('simulink/Sinks/Out1', [model '/Displacement'], 'Position', [550 85 580 105]);
add_line(model, 'ReferenceDisplacementStep/1', 'Error/1', 'autorouting', 'on');
add_line(model, 'FeedbackDelay/1', 'Error/2', 'autorouting', 'on');
add_line(model, 'Error/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'MassSpringDamper/1', 'autorouting', 'on');
add_line(model, 'MassSpringDamper/1', 'Displacement/1', 'autorouting', 'on');
add_line(model, 'MassSpringDamper/1', 'FeedbackDelay/1', 'autorouting', 'on');
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


function write_coupled_json(testDir, caseName, t_stop, dt, controllerFmu, plantFmu, refPort, measPort, controlPort, outputPort)
components.reference = struct('type', 'step', 'ports', struct(refPort, struct('initial', 0.0, 'step', 1.0, 'step_time', 0.0)));
components.controller = struct('type', 'fmu', 'file', controllerFmu);
components.plant = struct('type', 'fmu', 'file', plantFmu);
components.log = struct('type', 'logger', 'file', sprintf('results_%s.csv', caseName));
connections = {
    conn('reference', refPort, 'controller', refPort)
    conn('plant', outputPort, 'controller', measPort, true, 0.0)
    conn('controller', controlPort, 'plant', controlPort)
    conn('plant', outputPort, 'log', outputPort)
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

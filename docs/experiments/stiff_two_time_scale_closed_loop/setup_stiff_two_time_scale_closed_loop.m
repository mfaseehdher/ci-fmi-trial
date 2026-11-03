% setup_stiff_two_time_scale_closed_loop.m
% Coupled two-FMU validation case for a stiff plant with separated time scales.

testDir = pwd;
caseName = 'stiff_two_time_scale_closed_loop';
t_stop = 2;
dt = 0.0005;

plantModel = 'TwoTimeScale_Plant';
controllerModel = 'TwoTimeScale_Controller';
referenceModel = 'TwoTimeScale_ClosedLoop_Reference';

tauFast = 0.005;
tauSlow = 0.8;
Kp = 5.0;

create_two_time_scale_plant(plantModel, tauFast, tauSlow, t_stop, dt);
export_model(plantModel, testDir);

create_p_controller(controllerModel, Kp, t_stop, dt);
export_model(controllerModel, testDir);

create_two_time_scale_reference(referenceModel, tauFast, tauSlow, Kp, t_stop, dt);
run_reference(referenceModel, caseName, 'ProcessOutput');

write_coupled_json(testDir, caseName, t_stop, dt, ...
    'TwoTimeScale_Controller.fmu', 'TwoTimeScale_Plant.fmu', ...
    'ReferenceOutput', 'MeasuredOutput', 'ActuatorCommand', 'ProcessOutput');


function create_two_time_scale_plant(model, tauFast, tauSlow, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/ActuatorCommand'], 'Position', [35 90 65 110]);
add_block('simulink/Math Operations/Sum', [model '/FastError'], ...
    'Inputs', '+-', 'Position', [130 80 155 130]);
add_block('simulink/Math Operations/Gain', [model '/FastGain'], ...
    'Gain', num2str(1 / tauFast), 'Position', [205 90 275 120]);
add_block('simulink/Continuous/Integrator', [model '/FastActuatorState'], ...
    'InitialCondition', '0', 'Position', [330 90 360 120]);
add_block('simulink/Math Operations/Sum', [model '/SlowError'], ...
    'Inputs', '+-', 'Position', [425 80 450 130]);
add_block('simulink/Math Operations/Gain', [model '/SlowGain'], ...
    'Gain', num2str(1 / tauSlow), 'Position', [500 90 570 120]);
add_block('simulink/Continuous/Integrator', [model '/SlowProcessState'], ...
    'InitialCondition', '0', 'Position', [625 90 655 120]);
add_block('simulink/Sinks/Out1', [model '/ProcessOutput'], 'Position', [730 95 760 115]);
add_line(model, 'ActuatorCommand/1', 'FastError/1', 'autorouting', 'on');
add_line(model, 'FastActuatorState/1', 'FastError/2', 'autorouting', 'on');
add_line(model, 'FastError/1', 'FastGain/1', 'autorouting', 'on');
add_line(model, 'FastGain/1', 'FastActuatorState/1', 'autorouting', 'on');
add_line(model, 'FastActuatorState/1', 'SlowError/1', 'autorouting', 'on');
add_line(model, 'SlowProcessState/1', 'SlowError/2', 'autorouting', 'on');
add_line(model, 'SlowError/1', 'SlowGain/1', 'autorouting', 'on');
add_line(model, 'SlowGain/1', 'SlowProcessState/1', 'autorouting', 'on');
add_line(model, 'SlowProcessState/1', 'ProcessOutput/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_p_controller(model, Kp, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/In1', [model '/ReferenceOutput'], 'Position', [40 50 70 70]);
add_block('simulink/Sources/In1', [model '/MeasuredOutput'], 'Position', [40 130 70 150]);
add_block('simulink/Math Operations/Sum', [model '/OutputError'], ...
    'Inputs', '+-', 'Position', [140 75 165 125]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [230 85 290 115]);
add_block('simulink/Sinks/Out1', [model '/ActuatorCommand'], 'Position', [360 90 390 110]);
add_line(model, 'ReferenceOutput/1', 'OutputError/1', 'autorouting', 'on');
add_line(model, 'MeasuredOutput/1', 'OutputError/2', 'autorouting', 'on');
add_line(model, 'OutputError/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'ActuatorCommand/1', 'autorouting', 'on');
configure_model(model, t_stop, dt);
save_system(model);
end


function create_two_time_scale_reference(model, tauFast, tauSlow, Kp, t_stop, dt)
reset_model(model);
new_system(model);
add_block('simulink/Sources/Step', [model '/ReferenceOutputStep'], ...
    'Time', '0', 'Before', '0', 'After', '1', 'Position', [30 75 65 105]);
add_block('simulink/Discrete/Unit Delay', [model '/FeedbackDelay'], ...
    'InitialCondition', '0', 'SampleTime', num2str(dt), ...
    'Position', [130 190 180 230]);
add_block('simulink/Math Operations/Sum', [model '/OutputError'], ...
    'Inputs', '+-', 'Position', [130 80 155 130]);
add_block('simulink/Math Operations/Gain', [model '/Kp'], ...
    'Gain', num2str(Kp), 'Position', [215 90 275 120]);
add_block('simulink/Math Operations/Sum', [model '/FastError'], ...
    'Inputs', '+-', 'Position', [335 80 360 130]);
add_block('simulink/Math Operations/Gain', [model '/FastGain'], ...
    'Gain', num2str(1 / tauFast), 'Position', [410 90 480 120]);
add_block('simulink/Continuous/Integrator', [model '/FastActuatorState'], ...
    'InitialCondition', '0', 'Position', [535 90 565 120]);
add_block('simulink/Math Operations/Sum', [model '/SlowError'], ...
    'Inputs', '+-', 'Position', [625 80 650 130]);
add_block('simulink/Math Operations/Gain', [model '/SlowGain'], ...
    'Gain', num2str(1 / tauSlow), 'Position', [700 90 770 120]);
add_block('simulink/Continuous/Integrator', [model '/SlowProcessState'], ...
    'InitialCondition', '0', 'Position', [825 90 855 120]);
add_block('simulink/Sinks/Out1', [model '/ProcessOutput'], 'Position', [930 95 960 115]);
add_line(model, 'ReferenceOutputStep/1', 'OutputError/1', 'autorouting', 'on');
add_line(model, 'FeedbackDelay/1', 'OutputError/2', 'autorouting', 'on');
add_line(model, 'OutputError/1', 'Kp/1', 'autorouting', 'on');
add_line(model, 'Kp/1', 'FastError/1', 'autorouting', 'on');
add_line(model, 'FastActuatorState/1', 'FastError/2', 'autorouting', 'on');
add_line(model, 'FastError/1', 'FastGain/1', 'autorouting', 'on');
add_line(model, 'FastGain/1', 'FastActuatorState/1', 'autorouting', 'on');
add_line(model, 'FastActuatorState/1', 'SlowError/1', 'autorouting', 'on');
add_line(model, 'SlowProcessState/1', 'SlowError/2', 'autorouting', 'on');
add_line(model, 'SlowError/1', 'SlowGain/1', 'autorouting', 'on');
add_line(model, 'SlowGain/1', 'SlowProcessState/1', 'autorouting', 'on');
add_line(model, 'SlowProcessState/1', 'ProcessOutput/1', 'autorouting', 'on');
add_line(model, 'SlowProcessState/1', 'FeedbackDelay/1', 'autorouting', 'on');
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

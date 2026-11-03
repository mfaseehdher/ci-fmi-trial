% setup_inverted_pendulum_lqr_closed_loop.m
% Flagship coupled two-FMU validation case for an inverted pendulum.
%
% Reference:
%   InvertedPendulum_ClosedLoop_Reference.slx is a monolithic Simulink
%   closed-loop model.
%
% Co-simulation under test:
%   InvertedPendulum_LQR_Controller.fmu -> InvertedPendulum_Plant.fmu
%   with delayed plant-state feedback exchanged by generic.py.

testDir = pwd;
caseName = 'inverted_pendulum_lqr_closed_loop';
t_stop = 5;
dt = 0.001;

plantModel = 'InvertedPendulum_Plant';
controllerModel = 'InvertedPendulum_LQR_Controller';
referenceModel = 'InvertedPendulum_ClosedLoop_Reference';

% Linearized cart-pole parameters around the upright position.
M = 0.5;       % cart mass [kg]
m = 0.2;       % pendulum mass [kg]
b = 0.1;       % cart damping [N/(m/s)]
I = 0.006;     % pendulum inertia [kg*m^2]
g = 9.8;       % gravity [m/s^2]
l = 0.3;       % distance to pendulum center of mass [m]

p = I * (M + m) + M * m * l^2;
A = [0, 1, 0, 0; ...
     0, -((I + m*l^2) * b) / p, (m^2 * g * l^2) / p, 0; ...
     0, 0, 0, 1; ...
     0, -(m * l * b) / p, (m * g * l * (M + m)) / p, 0];
B = [0; (I + m*l^2) / p; 0; (m*l) / p];
C = eye(4);
D = zeros(4, 1);

% State vector: [cart position; cart velocity; pendulum angle; pendulum rate].
% A small initial angle disturbance makes the stabilizing behavior visible.
x0 = [0; 0; 0.08; 0];

% LQR-style state feedback gain for the linearized cart-pole model.
% The gain is hard-coded so the setup does not depend on Control System Toolbox.
K = [-70.7107, -37.8345, 105.5298, 20.9238];
forceLimit = 10;
referenceStep = 0.1;

create_inverted_pendulum_plant(plantModel, A, B, x0, t_stop, dt);
export_model(plantModel, testDir);

create_state_feedback_controller(controllerModel, K, forceLimit, t_stop, dt);
export_model(controllerModel, testDir);

create_reference_model(referenceModel, A, B, x0, K, forceLimit, referenceStep, t_stop, dt);
run_reference(referenceModel, caseName, {'CartPosition', 'PendulumAngle'});

write_coupled_json(testDir, caseName, t_stop, dt, referenceStep);


function create_inverted_pendulum_plant(model, A, B, x0, t_stop, dt)
reset_model(model);
new_system(model);

add_block('simulink/Sources/In1', [model '/Force'], ...
    'Position', [35 140 65 160]);

add_block('simulink/Math Operations/Gain', [model '/a22'], ...
    'Gain', num2str(A(2, 2), 16), ...
    'Position', [155 90 215 120]);
add_block('simulink/Math Operations/Gain', [model '/a23'], ...
    'Gain', num2str(A(2, 3), 16), ...
    'Position', [155 135 215 165]);
add_block('simulink/Math Operations/Gain', [model '/b2'], ...
    'Gain', num2str(B(2), 16), ...
    'Position', [155 180 215 210]);
add_block('simulink/Math Operations/Sum', [model '/CartAcceleration'], ...
    'Inputs', '+++', ...
    'Position', [260 130 290 185]);
add_block('simulink/Continuous/Integrator', [model '/CartVelocityState'], ...
    'InitialCondition', num2str(x0(2), 16), ...
    'Position', [340 135 370 165]);
add_block('simulink/Continuous/Integrator', [model '/CartPositionState'], ...
    'InitialCondition', num2str(x0(1), 16), ...
    'Position', [430 135 460 165]);

add_block('simulink/Math Operations/Gain', [model '/a42'], ...
    'Gain', num2str(A(4, 2), 16), ...
    'Position', [155 265 215 295]);
add_block('simulink/Math Operations/Gain', [model '/a43'], ...
    'Gain', num2str(A(4, 3), 16), ...
    'Position', [155 310 215 340]);
add_block('simulink/Math Operations/Gain', [model '/b4'], ...
    'Gain', num2str(B(4), 16), ...
    'Position', [155 355 215 385]);
add_block('simulink/Math Operations/Sum', [model '/PendulumAcceleration'], ...
    'Inputs', '+++', ...
    'Position', [260 305 290 360]);
add_block('simulink/Continuous/Integrator', [model '/PendulumRateState'], ...
    'InitialCondition', num2str(x0(4), 16), ...
    'Position', [340 310 370 340]);
add_block('simulink/Continuous/Integrator', [model '/PendulumAngleState'], ...
    'InitialCondition', num2str(x0(3), 16), ...
    'Position', [430 310 460 340]);

add_block('simulink/Sinks/Out1', [model '/CartPosition'], ...
    'Position', [535 125 570 145]);
add_block('simulink/Sinks/Out1', [model '/CartVelocity'], ...
    'Position', [535 165 570 185]);
add_block('simulink/Sinks/Out1', [model '/PendulumAngle'], ...
    'Position', [535 300 570 320]);
add_block('simulink/Sinks/Out1', [model '/PendulumRate'], ...
    'Position', [535 340 570 360]);

add_line(model, 'CartVelocityState/1', 'a22/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'a23/1', 'autorouting', 'on');
add_line(model, 'Force/1', 'b2/1', 'autorouting', 'on');
add_line(model, 'a22/1', 'CartAcceleration/1', 'autorouting', 'on');
add_line(model, 'a23/1', 'CartAcceleration/2', 'autorouting', 'on');
add_line(model, 'b2/1', 'CartAcceleration/3', 'autorouting', 'on');
add_line(model, 'CartAcceleration/1', 'CartVelocityState/1', 'autorouting', 'on');
add_line(model, 'CartVelocityState/1', 'CartPositionState/1', 'autorouting', 'on');

add_line(model, 'CartVelocityState/1', 'a42/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'a43/1', 'autorouting', 'on');
add_line(model, 'Force/1', 'b4/1', 'autorouting', 'on');
add_line(model, 'a42/1', 'PendulumAcceleration/1', 'autorouting', 'on');
add_line(model, 'a43/1', 'PendulumAcceleration/2', 'autorouting', 'on');
add_line(model, 'b4/1', 'PendulumAcceleration/3', 'autorouting', 'on');
add_line(model, 'PendulumAcceleration/1', 'PendulumRateState/1', 'autorouting', 'on');
add_line(model, 'PendulumRateState/1', 'PendulumAngleState/1', 'autorouting', 'on');

add_line(model, 'CartPositionState/1', 'CartPosition/1', 'autorouting', 'on');
add_line(model, 'CartVelocityState/1', 'CartVelocity/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'PendulumAngle/1', 'autorouting', 'on');
add_line(model, 'PendulumRateState/1', 'PendulumRate/1', 'autorouting', 'on');

configure_model(model, t_stop, dt);
save_system(model);
end


function create_state_feedback_controller(model, K, forceLimit, t_stop, dt)
reset_model(model);
new_system(model);

add_block('simulink/Sources/In1', [model '/ReferencePosition'], ...
    'Position', [35 45 65 65]);
add_block('simulink/Sources/In1', [model '/CartPosition'], ...
    'Position', [35 95 65 115]);
add_block('simulink/Sources/In1', [model '/CartVelocity'], ...
    'Position', [35 150 65 170]);
add_block('simulink/Sources/In1', [model '/PendulumAngle'], ...
    'Position', [35 205 65 225]);
add_block('simulink/Sources/In1', [model '/PendulumRate'], ...
    'Position', [35 260 65 280]);

add_block('simulink/Math Operations/Sum', [model '/PositionError'], ...
    'Inputs', '+-', ...
    'Position', [130 65 155 115]);
add_block('simulink/Math Operations/Gain', [model '/Kx'], ...
    'Gain', num2str(-K(1), 16), ...
    'Position', [210 75 280 105]);
add_block('simulink/Math Operations/Gain', [model '/Kxdot'], ...
    'Gain', num2str(-K(2), 16), ...
    'Position', [210 145 280 175]);
add_block('simulink/Math Operations/Gain', [model '/Ktheta'], ...
    'Gain', num2str(-K(3), 16), ...
    'Position', [210 200 280 230]);
add_block('simulink/Math Operations/Gain', [model '/Kthetadot'], ...
    'Gain', num2str(-K(4), 16), ...
    'Position', [210 255 280 285]);
add_block('simulink/Math Operations/Sum', [model '/ForceSum'], ...
    'Inputs', '++++', ...
    'Position', [345 145 375 225]);
add_block('simulink/Discontinuities/Saturation', [model '/ForceLimit'], ...
    'UpperLimit', num2str(forceLimit), ...
    'LowerLimit', num2str(-forceLimit), ...
    'Position', [425 165 480 205]);
add_block('simulink/Sinks/Out1', [model '/Force'], ...
    'Position', [535 175 570 195]);

add_line(model, 'CartPosition/1', 'PositionError/1', 'autorouting', 'on');
add_line(model, 'ReferencePosition/1', 'PositionError/2', 'autorouting', 'on');
add_line(model, 'PositionError/1', 'Kx/1', 'autorouting', 'on');
add_line(model, 'CartVelocity/1', 'Kxdot/1', 'autorouting', 'on');
add_line(model, 'PendulumAngle/1', 'Ktheta/1', 'autorouting', 'on');
add_line(model, 'PendulumRate/1', 'Kthetadot/1', 'autorouting', 'on');
add_line(model, 'Kx/1', 'ForceSum/1', 'autorouting', 'on');
add_line(model, 'Kxdot/1', 'ForceSum/2', 'autorouting', 'on');
add_line(model, 'Ktheta/1', 'ForceSum/3', 'autorouting', 'on');
add_line(model, 'Kthetadot/1', 'ForceSum/4', 'autorouting', 'on');
add_line(model, 'ForceSum/1', 'ForceLimit/1', 'autorouting', 'on');
add_line(model, 'ForceLimit/1', 'Force/1', 'autorouting', 'on');

configure_model(model, t_stop, dt);
save_system(model);
end


function create_reference_model(model, A, B, x0, K, forceLimit, referenceStep, t_stop, dt)
reset_model(model);
new_system(model);

add_block('simulink/Sources/Step', [model '/ReferenceStep'], ...
    'Time', '0', ...
    'Before', '0', ...
    'After', num2str(referenceStep), ...
    'Position', [35 50 70 80]);

add_block('simulink/Discrete/Unit Delay', [model '/CartPositionDelay'], ...
    'InitialCondition', num2str(x0(1), 16), ...
    'SampleTime', num2str(dt), ...
    'Position', [125 95 175 125]);
add_block('simulink/Discrete/Unit Delay', [model '/CartVelocityDelay'], ...
    'InitialCondition', num2str(x0(2), 16), ...
    'SampleTime', num2str(dt), ...
    'Position', [125 145 175 175]);
add_block('simulink/Discrete/Unit Delay', [model '/PendulumAngleDelay'], ...
    'InitialCondition', num2str(x0(3), 16), ...
    'SampleTime', num2str(dt), ...
    'Position', [125 195 175 225]);
add_block('simulink/Discrete/Unit Delay', [model '/PendulumRateDelay'], ...
    'InitialCondition', num2str(x0(4), 16), ...
    'SampleTime', num2str(dt), ...
    'Position', [125 245 175 275]);

add_block('simulink/Math Operations/Sum', [model '/PositionError'], ...
    'Inputs', '+-', ...
    'Position', [230 65 255 115]);
add_block('simulink/Math Operations/Gain', [model '/Kx'], ...
    'Gain', num2str(-K(1), 16), ...
    'Position', [305 75 365 105]);
add_block('simulink/Math Operations/Gain', [model '/Kxdot'], ...
    'Gain', num2str(-K(2), 16), ...
    'Position', [305 145 365 175]);
add_block('simulink/Math Operations/Gain', [model '/Ktheta'], ...
    'Gain', num2str(-K(3), 16), ...
    'Position', [305 195 365 225]);
add_block('simulink/Math Operations/Gain', [model '/Kthetadot'], ...
    'Gain', num2str(-K(4), 16), ...
    'Position', [305 245 365 275]);
add_block('simulink/Math Operations/Sum', [model '/ForceSum'], ...
    'Inputs', '++++', ...
    'Position', [425 145 455 225]);
add_block('simulink/Discontinuities/Saturation', [model '/ForceLimit'], ...
    'UpperLimit', num2str(forceLimit), ...
    'LowerLimit', num2str(-forceLimit), ...
    'Position', [500 165 555 205]);

add_block('simulink/Math Operations/Gain', [model '/a22'], ...
    'Gain', num2str(A(2, 2), 16), ...
    'Position', [610 75 670 105]);
add_block('simulink/Math Operations/Gain', [model '/a23'], ...
    'Gain', num2str(A(2, 3), 16), ...
    'Position', [610 115 670 145]);
add_block('simulink/Math Operations/Gain', [model '/b2'], ...
    'Gain', num2str(B(2), 16), ...
    'Position', [610 155 670 185]);
add_block('simulink/Math Operations/Sum', [model '/CartAcceleration'], ...
    'Inputs', '+++', ...
    'Position', [720 105 750 160]);
add_block('simulink/Continuous/Integrator', [model '/CartVelocityState'], ...
    'InitialCondition', num2str(x0(2), 16), ...
    'Position', [800 110 830 140]);
add_block('simulink/Continuous/Integrator', [model '/CartPositionState'], ...
    'InitialCondition', num2str(x0(1), 16), ...
    'Position', [890 110 920 140]);

add_block('simulink/Math Operations/Gain', [model '/a42'], ...
    'Gain', num2str(A(4, 2), 16), ...
    'Position', [610 285 670 315]);
add_block('simulink/Math Operations/Gain', [model '/a43'], ...
    'Gain', num2str(A(4, 3), 16), ...
    'Position', [610 325 670 355]);
add_block('simulink/Math Operations/Gain', [model '/b4'], ...
    'Gain', num2str(B(4), 16), ...
    'Position', [610 365 670 395]);
add_block('simulink/Math Operations/Sum', [model '/PendulumAcceleration'], ...
    'Inputs', '+++', ...
    'Position', [720 315 750 370]);
add_block('simulink/Continuous/Integrator', [model '/PendulumRateState'], ...
    'InitialCondition', num2str(x0(4), 16), ...
    'Position', [800 320 830 350]);
add_block('simulink/Continuous/Integrator', [model '/PendulumAngleState'], ...
    'InitialCondition', num2str(x0(3), 16), ...
    'Position', [890 320 920 350]);

add_block('simulink/Sinks/Out1', [model '/CartPosition'], ...
    'Position', [1010 105 1045 125]);
add_block('simulink/Sinks/Out1', [model '/PendulumAngle'], ...
    'Position', [1010 315 1045 335]);

add_line(model, 'CartPositionDelay/1', 'PositionError/1', 'autorouting', 'on');
add_line(model, 'ReferenceStep/1', 'PositionError/2', 'autorouting', 'on');
add_line(model, 'PositionError/1', 'Kx/1', 'autorouting', 'on');
add_line(model, 'CartVelocityDelay/1', 'Kxdot/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleDelay/1', 'Ktheta/1', 'autorouting', 'on');
add_line(model, 'PendulumRateDelay/1', 'Kthetadot/1', 'autorouting', 'on');
add_line(model, 'Kx/1', 'ForceSum/1', 'autorouting', 'on');
add_line(model, 'Kxdot/1', 'ForceSum/2', 'autorouting', 'on');
add_line(model, 'Ktheta/1', 'ForceSum/3', 'autorouting', 'on');
add_line(model, 'Kthetadot/1', 'ForceSum/4', 'autorouting', 'on');
add_line(model, 'ForceSum/1', 'ForceLimit/1', 'autorouting', 'on');

add_line(model, 'CartVelocityState/1', 'a22/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'a23/1', 'autorouting', 'on');
add_line(model, 'ForceLimit/1', 'b2/1', 'autorouting', 'on');
add_line(model, 'a22/1', 'CartAcceleration/1', 'autorouting', 'on');
add_line(model, 'a23/1', 'CartAcceleration/2', 'autorouting', 'on');
add_line(model, 'b2/1', 'CartAcceleration/3', 'autorouting', 'on');
add_line(model, 'CartAcceleration/1', 'CartVelocityState/1', 'autorouting', 'on');
add_line(model, 'CartVelocityState/1', 'CartPositionState/1', 'autorouting', 'on');

add_line(model, 'CartVelocityState/1', 'a42/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'a43/1', 'autorouting', 'on');
add_line(model, 'ForceLimit/1', 'b4/1', 'autorouting', 'on');
add_line(model, 'a42/1', 'PendulumAcceleration/1', 'autorouting', 'on');
add_line(model, 'a43/1', 'PendulumAcceleration/2', 'autorouting', 'on');
add_line(model, 'b4/1', 'PendulumAcceleration/3', 'autorouting', 'on');
add_line(model, 'PendulumAcceleration/1', 'PendulumRateState/1', 'autorouting', 'on');
add_line(model, 'PendulumRateState/1', 'PendulumAngleState/1', 'autorouting', 'on');

add_line(model, 'CartPositionState/1', 'CartPosition/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'PendulumAngle/1', 'autorouting', 'on');
add_line(model, 'CartPositionState/1', 'CartPositionDelay/1', 'autorouting', 'on');
add_line(model, 'CartVelocityState/1', 'CartVelocityDelay/1', 'autorouting', 'on');
add_line(model, 'PendulumAngleState/1', 'PendulumAngleDelay/1', 'autorouting', 'on');
add_line(model, 'PendulumRateState/1', 'PendulumRateDelay/1', 'autorouting', 'on');

configure_model(model, t_stop, dt);
save_system(model);
end


function export_model(model, testDir)
fprintf('  Exporting FMU: %s\n', model);
exportToFMU2CS(model, 'SaveDirectory', testDir);
close_system(model, 0);
end


function run_reference(model, caseName, signalNames)
fprintf('  Running MATLAB closed-loop reference: %s\n', model);
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
save_reference_csv(simOut, sprintf('%s_ref.csv', caseName), signalNames);
close_system(model, 0);
end


function save_reference_csv(simOut, fileName, signalNames)
t_out = simOut.tout(:);
raw = simOut.yout;
data = [];

if isa(raw, 'Simulink.SimulationData.Dataset')
    for i = 1:numel(signalNames)
        data(:, i) = raw{i}.Values.Data(:); %#ok<AGROW>
    end
else
    data = raw(:, 1:numel(signalNames));
end

n = min([numel(t_out), size(data, 1)]);
fid = fopen(fileName, 'w');
fprintf(fid, 'time');
for i = 1:numel(signalNames)
    fprintf(fid, ',%s', signalNames{i});
end
fprintf(fid, '\r\n');
for row = 1:n
    fprintf(fid, '%.10f', t_out(row));
    for col = 1:numel(signalNames)
        fprintf(fid, ',%.10f', data(row, col));
    end
    fprintf(fid, '\r\n');
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


function write_coupled_json(testDir, caseName, t_stop, dt, referenceStep)
components.reference = struct('type', 'step', 'ports', struct( ...
    'ReferencePosition', struct('initial', 0.0, 'step', referenceStep, 'step_time', 0.0)));
components.controller = struct('type', 'fmu', 'file', 'InvertedPendulum_LQR_Controller.fmu');
components.plant = struct('type', 'fmu', 'file', 'InvertedPendulum_Plant.fmu');
components.log = struct('type', 'logger', 'file', sprintf('results_%s.csv', caseName));

connections = {
    conn('reference', 'ReferencePosition', 'controller', 'ReferencePosition')
    conn('plant', 'CartPosition', 'controller', 'CartPosition', true, 0.0)
    conn('plant', 'CartVelocity', 'controller', 'CartVelocity', true, 0.0)
    conn('plant', 'PendulumAngle', 'controller', 'PendulumAngle', true, 0.08)
    conn('plant', 'PendulumRate', 'controller', 'PendulumRate', true, 0.0)
    conn('controller', 'Force', 'plant', 'Force')
    conn('plant', 'CartPosition', 'log', 'CartPosition')
    conn('plant', 'PendulumAngle', 'log', 'PendulumAngle')
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

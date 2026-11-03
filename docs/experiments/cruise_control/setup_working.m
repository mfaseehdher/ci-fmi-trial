% setup_cruise_control.m
% ccmodel = CTMS cruise control PID model
% Adds Outport (for FMU export) then simulates

model = 'ccmodel';

m = 1000;  b = 50;
assignin('base', 'm', m);
assignin('base', 'b', b);
Kp = 800;  Ki = 40;
assignin('base', 'Kp', Kp);
assignin('base', 'Ki', Ki);
u = 10;
assignin('base', 'u', u);
t_stop = 120;

load_system(model);

% Ensure model has an Outport for FMU export (one-time prep)
add_outports(model, 'Velocity');

set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver',     'ode4');
set_param(model, 'FixedStep',  '0.01');
set_param(model, 'StopTime',   num2str(t_stop));
set_param(model, 'SaveState',  'off');
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveFormat', 'Dataset');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

t_out = simOut.tout(:);
raw   = simOut.yout;
out   = raw{1}.Values.Data(:);

fid = fopen('ccmodel_ref.csv', 'w');
fprintf(fid, 'time,Velocity\r\n');
for i = 1:min(length(t_out),length(out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Cruise control done: %d rows\n', min(length(t_out),length(out)));

% setup_cruise_control.m
% ccmodel has an INTERNAL Step block - no external input needed

model = 'ccmodel';

m = 1000;  b = 50;
assignin('base', 'm', m);
assignin('base', 'b', b);

t_stop = 120;
assignin('base', 't_stop', t_stop);

load_system(model);
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver',     'ode4');
set_param(model, 'FixedStep',  '0.001');
set_param(model, 'StopTime',   num2str(t_stop));
% NO LoadExternalInput - model has internal Step block
set_param(model, 'LoadExternalInput', 'off');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

t_out   = simOut.tout(:);
raw     = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    vel_out = raw{1}.Values.Data(:);
else
    vel_out = raw(:,1);
end

fid = fopen('ccmodel_ref.csv', 'w');
fprintf(fid, 'time,Velocity\r\n');
for i = 1:min(length(t_out),length(vel_out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), vel_out(i));
end
fclose(fid);
fprintf('  Cruise control done: %d rows\n', length(t_out));

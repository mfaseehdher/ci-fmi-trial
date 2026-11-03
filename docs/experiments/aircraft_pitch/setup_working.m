% setup_aircraft_pitch.m
% aircraft_pitch_control = CTMS Aircraft Pitch
% Adds Outport (for FMU export) then simulates

model = 'aircraft_pitch_control';

A = [-0.313  56.7   0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];
K = [-0.6435 169.6950 7.0711];
assignin('base','A',A); assignin('base','B',B);
assignin('base','C',C); assignin('base','D',D);
assignin('base','K',K);
u = 0.2;
assignin('base','u',u);
t_stop = 10;

load_system(model);

% Ensure model has an Outport for FMU export
add_outports(model, 'PitchAngle');

set_param(model,'SolverType','Fixed-step');
set_param(model,'Solver','ode4');
set_param(model,'FixedStep','0.01');
set_param(model,'StopTime',num2str(t_stop));
set_param(model,'SaveState','off');
set_param(model,'SaveOutput','on');
set_param(model,'SaveFormat','Dataset');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

t_out = simOut.tout(:);
raw   = simOut.yout;
out   = raw{1}.Values.Data(:);

fid = fopen('aircraft_pitch_control_ref.csv', 'w');
fprintf(fid, 'time,PitchAngle\r\n');
for i = 1:min(length(t_out),length(out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Aircraft pitch done: %d rows\n', min(length(t_out),length(out)));

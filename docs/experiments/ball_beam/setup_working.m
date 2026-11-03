% setup_ball_beam.m
% ballBeam = CTMS Ball & Beam
% Adds Outport (for FMU export) then simulates

model = 'ballBeam';

m = 0.111;  R = 0.015;  g = -9.8;
L = 1.0;    d = 0.03;   J = 9.99e-6;
assignin('base','m',m); assignin('base','R',R);
assignin('base','g',g); assignin('base','L',L);
assignin('base','d',d); assignin('base','J',J);
u = 0.1;
assignin('base','u',u);
t_stop = 5;

load_system(model);

% Ensure model has an Outport for FMU export
add_outports(model, 'BallPosition');

set_param(model,'SolverType','Fixed-step');
set_param(model,'Solver','ode14x');
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

fid = fopen('ballBeam_ref.csv', 'w');
fprintf(fid, 'time,BallPosition\r\n');
for i = 1:min(length(t_out),length(out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Ball beam done: %d rows\n', min(length(t_out),length(out)));

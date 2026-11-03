% setup_ball_beam.m
% ballBeam has an INTERNAL Step block - no external input needed

model = 'ballBeam';

m = 0.111;  R = 0.015;  g = -9.8;
L = 1.0;    d = 0.03;   J = 9.99e-6;
assignin('base','m',m); assignin('base','R',R);
assignin('base','g',g); assignin('base','L',L);
assignin('base','d',d); assignin('base','J',J);

t_stop = 5;
assignin('base','t_stop',t_stop);

load_system(model);
set_param(model,'SolverType','Fixed-step');
set_param(model,'Solver','ode14x');
set_param(model,'FixedStep','0.001');
set_param(model,'StopTime',num2str(t_stop));
set_param(model,'LoadExternalInput','off');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

t_out = simOut.tout(:);
raw   = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    out = raw{1}.Values.Data(:);
else
    out = raw(:,1);
end

fid = fopen('ballBeam_ref.csv', 'w');
fprintf(fid, 'time,BallPosition\r\n');
for i = 1:min(length(t_out),length(out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Ball beam done: %d rows\n', length(t_out));

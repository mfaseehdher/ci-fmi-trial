% setup_suspension.m
% suspmod.mdl - suspension model
% Check if it has Inport; old script used external input

model = 'suspmod';

M1 = 2500;  M2 = 320;
K1 = 80000; K2 = 500000;
b1 = 350;   b2 = 15020;
assignin('base','M1',M1); assignin('base','M2',M2);
assignin('base','K1',K1); assignin('base','K2',K2);
assignin('base','b1',b1); assignin('base','b2',b2);

t_stop = 5;
t = (0:0.001:t_stop)';
u = 0.1 * ones(size(t));
assignin('base','t',t);
assignin('base','u',u);

load_system(model);
set_param(model,'SolverType','Fixed-step');
set_param(model,'Solver','ode4');
set_param(model,'FixedStep','0.001');
set_param(model,'StopTime',num2str(t_stop));

% Check if model has root inport - decide external input
inports = find_system(model, 'SearchDepth', 1, 'BlockType', 'Inport');
if ~isempty(inports)
    set_param(model,'LoadExternalInput','on');
    set_param(model,'ExternalInput','[t, u]');
    fprintf('  Suspension: using external input\n');
else
    set_param(model,'LoadExternalInput','off');
    fprintf('  Suspension: using internal source\n');
end
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

t_out = simOut.tout(:);
raw   = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    out = raw{1}.Values.Data(:);
else
    out = raw(:,1);
end

fid = fopen('suspmod_ref.csv', 'w');
fprintf(fid, 'time,BodyDisplacement\r\n');
for i = 1:min(length(t_out),length(out))
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Suspension done: %d rows\n', length(t_out));

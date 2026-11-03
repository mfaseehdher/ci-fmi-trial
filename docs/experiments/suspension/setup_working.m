% Working variant of the earlier suspension setup. The original is retained.
model = 'suspmod';

% The old .mdl block expressions use lowercase names.
m1 = 2500; m2 = 320;
k1 = 80000; k2 = 500000;
b1 = 350; b2 = 15020;
assignin('base', 'm1', m1); assignin('base', 'm2', m2);
assignin('base', 'k1', k1); assignin('base', 'k2', k2);
assignin('base', 'b1', b1); assignin('base', 'b2', b2);

t_stop = 5;
t = (0:0.01:t_stop)';
u = 0.1 * ones(size(t));
assignin('base', 't', t);
assignin('base', 'u', u);

load_system(model);
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.01', 'StopTime', num2str(t_stop), ...
    'LoadExternalInput', 'on', 'ExternalInput', '[t, u]', ...
    'SaveOutput', 'on', 'SaveFormat', 'Dataset');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
t_out = simOut.tout(:);
raw = simOut.yout;
if isa(raw, 'Simulink.SimulationData.Dataset')
    out = raw{1}.Values.Data(:);
else
    out = raw(:, 1);
end
if numel(t_out) ~= numel(out)
    error('Suspension reference time and output lengths differ.');
end

fid = fopen('suspmod_ref.csv', 'w');
if fid < 0, error('Cannot write suspension reference.'); end
fprintf(fid, 'time,BodyDisplacement\r\n');
for i = 1:numel(t_out)
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('Suspension reference saved: %d rows.\n', numel(t_out));

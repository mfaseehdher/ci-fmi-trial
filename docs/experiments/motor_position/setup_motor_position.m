% setup_motor_position.m
model = 'Motor_Pos';

K = 0.01;  J = 0.01;  b = 0.1;  R = 1.0;  L = 0.5;
assignin('base', 'K', K);
assignin('base', 'J', J);
assignin('base', 'b', b);
assignin('base', 'R', R);
assignin('base', 'L', L);

t_stop = 10;
t = (0 : 0.001 : t_stop)';
u = 1.0 * ones(size(t));
assignin('base', 't', t);
assignin('base', 'u', u);

load_system(model);
set_param(model, 'SolverType',        'Fixed-step');
set_param(model, 'Solver',            'ode14x');
set_param(model, 'FixedStep',         '0.001');
set_param(model, 'StopTime',          num2str(t_stop));
set_param(model, 'LoadExternalInput', 'on');
set_param(model, 'ExternalInput',     '[t, u]');
save_system(model);

sim(model);

t_out  = tout;
pos_out = yout{1}.Values.Data(:);

fid = fopen('Motor_Pos_ref.csv', 'w');
fprintf(fid, 'time,Position\r\n');
for i = 1:length(t_out)
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), pos_out(i));
end
fclose(fid);
fprintf('  Motor position workspace ready\n');
fprintf('  Reference saved: Motor_Pos_ref.csv (%d rows)\n', length(t_out));

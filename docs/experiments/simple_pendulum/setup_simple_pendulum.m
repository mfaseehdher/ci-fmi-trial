% setup_simple_pendulum.m
% CTMS Simple Pendulum - file: pendulumSIM.slx
% Has two Integrators needing initial conditions:
%   theta_ic     = initial angle
%   theta_dot_ic = initial angular velocity

model = 'pendulumSIM';

% Parameters (CTMS simple pendulum)
M = 0.380;
m = 0.095;
l = 0.43;
b = 0.003;
g = 9.81;
assignin('base','M',M); assignin('base','m',m);
assignin('base','l',l); assignin('base','b',b);
assignin('base','g',g);

% Initial conditions for the two Integrators
theta_ic     = 0.423;   % initial angle (rad) - released from ~24 degrees
theta_dot_ic = 0;       % initial angular velocity (rad/s) - starts at rest
assignin('base','theta_ic',     theta_ic);
assignin('base','theta_dot_ic', theta_dot_ic);

% Derived quantities some CTMS pendulum models use
lG = l;
IO = M * l^2;
assignin('base','lG',lG);
assignin('base','IO',IO);

t_stop = 10;

load_system(model);
add_outport_from_to_workspace(model, 'theta_nl', 'Angle');
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver',     'ode4');
set_param(model, 'FixedStep',  '0.01');
set_param(model, 'StopTime',   num2str(t_stop));
set_param(model, 'SaveState',  'off');
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveFormat', 'Dataset');
save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

% Show what the model produced
who_vars = simOut.who;
fprintf('=== simOut contents ===\n');
for i = 1:numel(who_vars)
    fprintf('  Available: %s\n', who_vars{i});
end

t_out = simOut.tout(:);

% This model uses To Workspace blocks:
%   theta_nl  = nonlinear pendulum angle (what we want for nonlinear category)
%   theta_lin = linearized angle
%   time      = model's own time vector
out = [];

% Preferred: nonlinear angle
if ismember('theta_nl', who_vars)
    raw = simOut.get('theta_nl');
    out = squeeze(raw);
    out = out(:);
    fprintf('  Got theta_nl (nonlinear angle), %d points\n', numel(out));
end

% Use the model's own time vector if present (matches the To Workspace data)
if ismember('time', who_vars)
    tvar = squeeze(simOut.get('time'));
    t_out = tvar(:);
end

% Fallbacks if theta_nl not present
if isempty(out) && ismember('theta_lin', who_vars)
    raw = simOut.get('theta_lin');
    out = squeeze(raw); out = out(:);
    fprintf('  Got theta_lin (linear angle), %d points\n', numel(out));
end

if isempty(out) && ismember('yout', who_vars)
    raw = simOut.yout;
    if isa(raw, 'Simulink.SimulationData.Dataset')
        out = raw{1}.Values.Data(:);
    elseif isnumeric(raw)
        out = raw(:,1);
    end
end

if isempty(out)
    error('No output found (expected theta_nl from To Workspace).');
end

% Guard: if time and out lengths differ, align by trimming
n = min(length(t_out), length(out));
t_out = t_out(1:n);
out   = out(1:n);

fid = fopen('pendulumSIM_ref.csv', 'w');
fprintf(fid, 'time,Angle\r\n');
for i = 1:n
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Simple pendulum done: %d rows (nonlinear angle)\n', n);

function add_outport_from_to_workspace(model, variableName, outName)

blocks = find_system(model, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'BlockType', 'ToWorkspace');

target = '';
for i = 1:numel(blocks)
    if strcmp(get_param(blocks{i}, 'VariableName'), variableName)
        target = blocks{i};
        break;
    end
end

if isempty(target)
    error('Could not find To Workspace block with VariableName "%s".', variableName);
end

ph = get_param(target, 'PortHandles');
inLine = get_param(ph.Inport(1), 'Line');

if inLine == -1
    error('To Workspace block "%s" input is not connected.', target);
end

srcPort = get_param(inLine, 'SrcPortHandle');
outportPath = [model '/' outName];

if isempty(find_system(model, 'SearchDepth', 1, 'Name', outName))
    add_block('simulink/Sinks/Out1', outportPath);
end

outPh = get_param(outportPath, 'PortHandles');

try
    add_line(model, srcPort, outPh.Inport(1), 'autorouting', 'on');
catch
end

save_system(model);
fprintf('Added Outport "%s" from To Workspace "%s"\n', outName, variableName);

end

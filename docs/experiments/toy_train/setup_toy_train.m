% setup_toy_train.m
% CTMS Toy Train (Introduction example) - two masses + spring
% Equations:
%   M1*x1dd = F - k(x1-x2) - mu*M1*g*x1d
%   M2*x2dd = k(x1-x2) - mu*M2*g*x2d
% Output of interest: x1_dot (engine velocity), via Scope -> add Outport
% Input: internal Signal Generator (square, 0.001 Hz, amp -1) -- NOT a root Inport
%
% CHANGE 'MODELNAME' to your actual downloaded filename (without .slx)

model = 'train';

% Parameters (from CTMS page)
M1 = 1;
M2 = 0.5;
k  = 1;
F  = 1;
mu = 0.02;
g  = 9.8;
assignin('base','M1',M1); assignin('base','M2',M2);
assignin('base','k',k);   assignin('base','F',F);
assignin('base','mu',mu); assignin('base','g',g);

t_stop = 1000;

load_system(model);

% This model outputs via Scope. Add an Outport on the x1_dot signal
% for FMU export, named "EngineVelocity".
add_outports(model, 'EngineVelocity');

set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver',     'ode4');
set_param(model, 'FixedStep',  '0.1');     % coarse: 1000s -> 10001 rows
set_param(model, 'StopTime',   num2str(t_stop));
set_param(model, 'SaveState',  'off');
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveFormat', 'Dataset');

% Internal Signal Generator drives F -- do NOT set LoadExternalInput

save_system(model);

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');

who_vars = simOut.who;
fprintf('=== simOut contents ===\n');
for i = 1:numel(who_vars)
    fprintf('  Available: %s\n', who_vars{i});
end

t_out = simOut.tout(:);
out = [];

if ismember('yout', who_vars)
    raw = simOut.yout;
    if isa(raw, 'Simulink.SimulationData.Dataset')
        fprintf('  yout has %d signal(s)\n', raw.numElements);
        out = raw{1}.Values.Data(:);
    elseif isnumeric(raw)
        out = raw(:,1);
    end
end

if isempty(out) && ismember('logsout', who_vars)
    logs = simOut.logsout;
    try
        if logs.numElements > 0
            elem  = logs.getElement(1);
            out   = elem.Values.Data(:);
            t_out = elem.Values.Time(:);
            fprintf('  Got data from logsout\n');
        end
    catch
    end
end

if isempty(out)
    error(['No output found. Run find_system to check Outport/Scope. ' ...
           'The add_outports call should have added an Outport.']);
end

n = min(length(t_out), length(out));
fid = fopen(sprintf('%s_ref.csv', model), 'w');
fprintf(fid, 'time,EngineVelocity\r\n');
for i = 1:n
    fprintf(fid, '%.10f,%.10f\r\n', t_out(i), out(i));
end
fclose(fid);
fprintf('  Toy train done: %d rows\n', n);

function add_outports(model, outName)

load_system(model);

scopes = find_system(model, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'BlockType', 'Scope');

fprintf('Found %d scope(s) in %s\n', numel(scopes), model);

if isempty(scopes)
    return;
end

scope = scopes{1};
ph = get_param(scope, 'PortHandles');
inLine = get_param(ph.Inport(1), 'Line');

if inLine == -1
    error('Scope input is not connected.');
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
    % Line may already exist
end

save_system(model);
fprintf('Added Outport "%s" connected to scope signal\n', outName);

end

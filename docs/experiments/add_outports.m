function add_outports(model_name, output_name)
% Expose the sole top-level Scope signal on a working copy of a model.
load_system(model_name);
existing = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Outport');
if ~isempty(existing)
    error('%s already has a top-level Outport; select it explicitly.', model_name);
end
scopes = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Scope');
if numel(scopes) ~= 1
    error('%s has %d top-level Scopes; cannot choose an output automatically.', ...
        model_name, numel(scopes));
end
scope_ports = get_param(scopes{1}, 'PortHandles');
if numel(scope_ports.Inport) ~= 1
    error('%s has %d Scope inputs; select one explicitly.', ...
        model_name, numel(scope_ports.Inport));
end
line_handle = get_param(scope_ports.Inport(1), 'Line');
if line_handle == -1
    error('Scope input in %s is not connected.', model_name);
end
source_port = get_param(line_handle, 'SrcPortHandle');
scope_pos = get_param(scopes{1}, 'Position');
out_block = [model_name '/' output_name];
add_block('simulink/Sinks/Out1', out_block, ...
    'Position', [scope_pos(1)+200 scope_pos(2) ...
                 scope_pos(1)+230 scope_pos(2)+30]);
out_ports = get_param(out_block, 'PortHandles');
add_line(model_name, source_port, out_ports.Inport(1), 'autorouting', 'on');
set_param(model_name, 'SaveOutput', 'on', 'SaveFormat', 'Dataset');
fprintf('Exposed %s Scope signal as Outport %s.\n', model_name, output_name);
end

function prepare_simulink_experiment(experiment_dir, build_dir)
% Prepare one copied Simulink model without changing the checked-in source.
experiment_dir = char(experiment_dir);
manifest = jsondecode(fileread(fullfile(experiment_dir, 'case.json')));
addpath(fileparts(mfilename('fullpath')), '-begin');
if nargin < 2, build_dir = fullfile(experiment_dir, 'build'); end
build_dir = char(build_dir);
if ~exist(build_dir, 'dir'), mkdir(build_dir); end

model_file = char(manifest.model);
[~, model_name] = fileparts(model_file);
copyfile(fullfile(experiment_dir, model_file), fullfile(build_dir, model_file));
setup_file = '';
if isfield(manifest, 'setup')
    setup_file = char(manifest.setup);
    copyfile(fullfile(experiment_dir, setup_file), fullfile(build_dir, setup_file));
end

previous_dir = pwd;
cleanup = onCleanup(@() cd(previous_dir));
cd(build_dir);
status = struct('model', model_name, 'setup_file', setup_file, 'setup_ok', false, ...
    'setup_error', '', 'export_ok', false, 'export_error', '', ...
    'inputs', {{}}, 'outputs', {{}}, 'fixed_step', '', ...
    'stop_time', '', 'reference_file', '', 'stimulus_file', '', ...
    'fmu_file', '');

try
    if isfield(manifest, 'setup')
        run(setup_file);
    end
    status.setup_ok = true;
catch err
    status.setup_error = getReport(err, 'basic', 'hyperlinks', 'off');
end

try
    load_system(model_name);
    inports = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Inport');
    outports = find_system(model_name, 'SearchDepth', 1, 'BlockType', 'Outport');
    status.inputs = port_names(inports);
    status.outputs = port_names(outports);
    status.fixed_step = get_param(model_name, 'FixedStep');
    status.stop_time = get_param(model_name, 'StopTime');
catch err
    status.export_error = getReport(err, 'basic', 'hyperlinks', 'off');
    write_status(build_dir, status);
    return;
end

references = dir(fullfile(build_dir, '*_ref.csv'));
if numel(references) == 1
    status.reference_file = references(1).name;
end

% A setup may provide a single external input as workspace vectors t and u.
if numel(status.inputs) == 1 && evalin('base', 'exist(''t'',''var'')') && ...
        evalin('base', 'exist(''u'',''var'')')
    t = evalin('base', 't(:)');
    u = evalin('base', 'u(:)');
    if isnumeric(t) && isnumeric(u) && numel(t) == numel(u)
        input_trace = table(t, u, 'VariableNames', {'time', status.inputs{1}});
        writetable(input_trace, fullfile(build_dir, 'stimulus.csv'));
        status.stimulus_file = 'stimulus.csv';
    end
end

try
    exportToFMU2CS(model_name, 'SaveDirectory', build_dir);
    fmu_path = fullfile(build_dir, [model_name '.fmu']);
    if ~exist(fmu_path, 'file')
        error('FMU export returned without creating %s', fmu_path);
    end
    status.fmu_file = [model_name '.fmu'];
    status.export_ok = true;
catch err
    status.export_error = getReport(err, 'basic', 'hyperlinks', 'off');
end

if bdIsLoaded(model_name), close_system(model_name, 0); end
write_status(build_dir, status);
end

function names = port_names(paths)
names = cell(size(paths));
for i = 1:numel(paths)
    names{i} = get_param(paths{i}, 'Name');
end
end

function write_status(build_dir, status)
fid = fopen(fullfile(build_dir, 'matlab_status.json'), 'w');
if fid < 0, error('Cannot write MATLAB status'); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(status, 'PrettyPrint', true));
end

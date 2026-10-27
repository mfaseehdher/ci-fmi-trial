function generate_reference_traces()
% Generate sample-aligned reference CSVs by simulating Simulink models.

experiment_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(experiment_dir, 'models');
reference_dir = fullfile(experiment_dir, 'references');
if ~exist(reference_dir, 'dir'), mkdir(reference_dir); end

adder_name = 'adder';
load_system(fullfile(model_dir, [adder_name '.slx']));
run_adder_reference('01_two_inputs', 2, 3, 3);
run_adder_reference('02_fan_out', 2, 2, 3);
run_adder_reference('08_two_fmu_chain', 5, 4, 3);
run_adder_reference('09_branch_and_merge', 3, 5, 3);
close_system(adder_name, 0);

feedback_name = 'reference_feedback';
create_feedback_model(feedback_name, model_dir);
run_feedback_reference('03_delayed_feedback', 1, 1, 4);
run_feedback_reference('05_weighted_delayed_feedback', 1, 0.5, 4);
run_feedback_reference('10_two_fmu_delayed_loop', 3, 1, 4);
close_system(feedback_name, 0);

    function run_adder_reference(case_name, input_a, input_b, count)
        time = (0:count)' * 0.1;
        data = Simulink.SimulationData.Dataset;
        data = setElement(data, 1, timeseries(input_a * ones(size(time)), time));
        data = setElement(data, 2, timeseries(input_b * ones(size(time)), time));
        input = Simulink.SimulationInput(adder_name);
        input = setExternalInput(input, data);
        input = setModelParameter(input, 'StopTime', num2str(count * 0.1), ...
            'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
        output = sim(input);
        write_reference(case_name, output, count);
    end

    function run_feedback_reference(case_name, bias, gain, count)
        input = Simulink.SimulationInput(feedback_name);
        input = setBlockParameter(input, [feedback_name '/Bias'], ...
            'Value', num2str(bias));
        input = setBlockParameter(input, [feedback_name '/LoopGain'], ...
            'Gain', num2str(gain));
        input = setModelParameter(input, 'StopTime', num2str(count * 0.1), ...
            'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
        output = sim(input);
        write_reference(case_name, output, count);
    end

    function write_reference(case_name, output, count)
        dataset = output.get('yout');
        trace = dataset.getElement(1).Values;
        values = double(trace.Data(:));
        if numel(values) < count
            error('Reference %s has only %d samples', case_name, numel(values));
        end
        table_data = table((1:count)' * 0.1, values(1:count), ...
            'VariableNames', {'time', 'sum'});
        writetable(table_data, fullfile(reference_dir, [case_name '.csv']));
        fprintf('Reference %s: %s\n', case_name, mat2str(values(1:count)'));
    end
end


function create_feedback_model(model_name, model_dir)
if bdIsLoaded(model_name), close_system(model_name, 0); end
new_system(model_name);
add_block('simulink/Sources/Constant', [model_name '/Bias'], ...
    'Value', '1', 'Position', [40 40 75 70]);
add_block('simulink/Discrete/Unit Delay', [model_name '/PreviousOutput'], ...
    'InitialCondition', '0', 'SampleTime', '0.1', ...
    'Position', [190 145 225 175]);
add_block('simulink/Math Operations/Gain', [model_name '/LoopGain'], ...
    'Gain', '1', 'Position', [100 135 145 185]);
add_block('simulink/Math Operations/Sum', [model_name '/Add'], ...
    'Inputs', '++', 'Position', [190 45 220 105]);
add_block('simulink/Sinks/Out1', [model_name '/Out'], ...
    'Position', [295 65 325 85]);
add_line(model_name, 'Bias/1', 'Add/1');
add_line(model_name, 'PreviousOutput/1', 'LoopGain/1');
add_line(model_name, 'LoopGain/1', 'Add/2');
add_line(model_name, 'Add/1', 'Out/1');
add_line(model_name, 'Add/1', 'PreviousOutput/1');
set_param(model_name, 'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1', ...
    'StartTime', '0', 'StopTime', '0.4');
save_system(model_name, fullfile(model_dir, [model_name '.slx']));
end

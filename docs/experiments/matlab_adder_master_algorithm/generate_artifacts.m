function generate_artifacts()
% Build the FMUs and independent Simulink reference traces for this experiment.

experiment_dir = fileparts(mfilename('fullpath'));
addpath(experiment_dir);
create_and_export_adder();
create_and_export_weighted_adder();
create_and_export_network_adders();
generate_reference_traces();
end

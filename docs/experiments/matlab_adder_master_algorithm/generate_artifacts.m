function generate_artifacts()
% Build shared FMUs and Simulink reference traces for the adder experiments.

experiment_dir = fileparts(mfilename('fullpath'));
addpath(experiment_dir);
create_and_export_adder();
create_and_export_weighted_adder();
create_and_export_network_adders();
create_and_export_loop_adders();
generate_reference_traces();
end

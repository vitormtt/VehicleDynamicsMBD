function filepath = save_simulation_results(sim_data, output_dir)
%SAVE_SIMULATION_RESULTS Salva com nomenclatura padronizada
if nargin < 2 || isempty(output_dir)
    output_dir = fullfile('results', '5dof', ...
                         sim_data.Metadata.controller, ...
                         sim_data.Metadata.maneuver);
end

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

timestamp_str = datestr(sim_data.Metadata.timestamp, 'yyyy-mm-dd_HHMMSS');
vehicle_short = strrep(sim_data.Metadata.vehicle, ' ', '');
vehicle_short = strrep(vehicle_short, 'Chevrolet', 'Chev');
controller_str = sim_data.Metadata.controller;
maneuver_str = sim_data.Metadata.maneuver;
velocity_str = sprintf('%dkmh', round(sim_data.Metadata.velocity_kmh));

filename = sprintf('%s_%s_%s_%s_%s.mat', ...
    timestamp_str, vehicle_short, controller_str, maneuver_str, velocity_str);
filepath = fullfile(output_dir, filename);

% Ensure sim_data is structured so both metadata and metrics can be read by
% compare_simulation_metrics correctly.
metadata = sim_data.Metadata;
if isfield(sim_data, 'Metrics')
    metadata.metrics = sim_data.Metrics;
end

save(filepath, 'sim_data', 'metadata', '-v7.3');
file_info = dir(filepath);
fprintf('   ✅ Salvo: %s\n', filepath);
fprintf('   Tamanho: %.2f MB\n', file_info.bytes / 1e6);
end

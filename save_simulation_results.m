function filepath = save_simulation_results(sim_data, output_dir)
%SAVE_SIMULATION_RESULTS Salva resultados com nomenclatura padronizada
%
% Formato: YYYY-MM-DD_HHMMSS_<vehicle>_<controller>_<maneuver>_<velocity>kmh.mat
%
% Inputs:
%   sim_data   - struct gerado por create_simulation_structure
%   output_dir - (opcional) diretório de saída customizado
%
% Output:
%   filepath - caminho completo do arquivo salvo
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Criar diretório de saída
if nargin < 2 || isempty(output_dir)
    output_dir = fullfile('results', '5dof', ...
                         sim_data.Metadata.controller, ...
                         sim_data.Metadata.maneuver);
end

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Gerar nome do arquivo
timestamp_str = datestr(sim_data.Metadata.timestamp, 'yyyy-mm-dd_HHMMSS');
vehicle_short = strrep(sim_data.Metadata.vehicle, ' ', '');
vehicle_short = strrep(vehicle_short, 'Chevrolet', 'Chev'); % Encurtar
controller_str = sim_data.Metadata.controller;
maneuver_str = sim_data.Metadata.maneuver;
velocity_str = sprintf('%dkmh', round(sim_data.Metadata.velocity_kmh));

filename = sprintf('%s_%s_%s_%s_%s.mat', ...
    timestamp_str, vehicle_short, controller_str, maneuver_str, velocity_str);

filepath = fullfile(output_dir, filename);

%% Salvar com compressão
fprintf('   Salvando resultados...\n');
save(filepath, 'sim_data', '-v7.3'); % HDF5-based (suporta >2GB)

file_info = dir(filepath);
fprintf('   ✅ Salvo: %s\n', filepath);
fprintf('   Tamanho: %.2f MB\n', file_info.bytes / 1e6);

end

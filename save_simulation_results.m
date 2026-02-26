function filepath = save_simulation_results(simOut, metadata, output_dir)
%SAVE_SIMULATION_RESULTS Salva resultados da simulação (simOut)
%
% Formato: YYYY-MM-DD_HHMMSS_<vehicle>_<controller>_<maneuver>_<velocity>kmh.mat
%
% Inputs:
%   simOut     - objeto Simulink.SimulationOutput gerado pelo comando sim()
%   metadata   - struct contendo informacoes da simulacao (vehicle, controller, etc)
%   output_dir - (opcional) diretório de saída customizado
%
% Output:
%   filepath - caminho completo do arquivo salvo
%
% Autor: Vitor Yukio - UnB/PIBIC
% Atualizado: 26/02/2026 (MBD Compliant)

%% Criar diretório de saída
if nargin < 3 || isempty(output_dir)
    output_dir = fullfile('results', '5dof', ...
                         metadata.controller, ...
                         metadata.maneuver);
end

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Gerar nome do arquivo
timestamp_str = datestr(metadata.timestamp, 'yyyy-mm-dd_HHMMSS');
vehicle_short = strrep(metadata.vehicle, ' ', '');
vehicle_short = strrep(vehicle_short, 'Chevrolet', 'Chev'); % Encurtar
controller_str = metadata.controller;
maneuver_str = metadata.maneuver;
velocity_str = sprintf('%dkmh', round(metadata.velocity_kmh));

filename = sprintf('%s_%s_%s_%s_%s.mat', ...
    timestamp_str, vehicle_short, controller_str, maneuver_str, velocity_str);

filepath = fullfile(output_dir, filename);

%% Estruturar dados para salvar
sim_data = struct();
sim_data.Metadata = metadata;
sim_data.simOut = simOut;

%% Salvar com compressão
fprintf('   Salvando resultados...\n');
save(filepath, 'sim_data', '-v7.3'); % HDF5-based (suporta >2GB)

file_info = dir(filepath);
fprintf('   Tamanho: %.2f MB\n', file_info.bytes / 1e6);

end

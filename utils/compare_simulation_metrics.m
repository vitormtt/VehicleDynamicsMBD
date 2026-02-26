function compare_simulation_metrics()
%COMPARE_SIMULATION_METRICS Lê os resultados salvos e exibe uma tabela de comparação
%
% Autor: Vitor Yukio - UnB/PIBIC

    results_dir = fullfile('results', '5dof');
    
    if ~exist(results_dir, 'dir')
        fprintf('Nenhum resultado encontrado no diretorio: %s\n', results_dir);
        return;
    end
    
    % Busca todos os arquivos .mat recursivamente na pasta 5dof
    files = dir(fullfile(results_dir, '**', '*.mat'));
    
    if isempty(files)
        fprintf('Nenhum arquivo .mat encontrado.\n');
        return;
    end
    
    fprintf('\n');
    fprintf('========================================================================================\n');
    fprintf('                           TABELA DE COMPARAÇÃO DE RESULTADOS                           \n');
    fprintf('========================================================================================\n');
    fprintf('| %-10s | %-10s | %-8s | %-12s | %-12s | %-10s |\n', 'Controller', 'Maneuver', 'Vel(kmh)', 'Max Roll(°)', 'RMS Roll(°)', 'Time (s)');
    fprintf('|------------|------------|----------|--------------|--------------|------------|\n');
    
    for i = 1:length(files)
        try
            data = load(fullfile(files(i).folder, files(i).name));
            
            % Tenta encontrar metadados de diferentes formas dependendo da versao do script
            md = [];
            if isfield(data, 'metadata')
                md = data.metadata;
            elseif isfield(data, 'sim_data') && isfield(data.sim_data, 'Metadata')
                md = data.sim_data.Metadata;
            end
            
            if ~isempty(md)
                ctrl = md.controller;
                man = md.maneuver;
                vel = md.velocity_kmh;
                
                if isfield(md, 'metrics')
                    max_roll = md.metrics.max_roll_deg;
                    rms_roll = md.metrics.rms_roll_deg;
                    if isfield(md.metrics, 'execution_time_s')
                        exec_time = md.metrics.execution_time_s;
                    else
                        exec_time = NaN;
                    end
                else
                    max_roll = NaN;
                    rms_roll = NaN;
                    exec_time = NaN;
                end
                
                fprintf('| %-10s | %-10s | %-8.1f | %-12.4f | %-12.4f | %-10.2f |\n', ...
                    ctrl, man, vel, max_roll, rms_roll, exec_time);
            end
        catch
            % Ignora arquivos que nao sao de resultado do simulador
        end
    end
    fprintf('========================================================================================\n\n');
end

function fig = plot_5dof_results_v2(simOut, metadata)
%PLOT_5DOF_RESULTS_V2 Visualização profissional (Publication-ready)
%
% Input: 
%   simOut   - objeto Simulink.SimulationOutput
%   metadata - struct com metadados da simulação
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Extrair dados do simOut
try
    has_logsout = isprop(simOut, 'logsout') && ~isempty(simOut.logsout);
    has_xout = isprop(simOut, 'xout') && ~isempty(simOut.xout);
    has_results = isprop(simOut, 'results') && ~isempty(simOut.results);

    % Tenta logsout primeiro
    if has_logsout && isa(simOut.logsout, 'Simulink.SimulationData.Dataset') && simOut.logsout.numElements > 0 && ~isempty(simOut.logsout.find('phi'))
        logs = simOut.logsout;
        time = logs.get(1).Values.Time;
        
        try
            phi = logs.getElement('phi').Values.Data;
            r = logs.getElement('r').Values.Data;
            beta = logs.getElement('beta').Values.Data;
            p = logs.getElement('p').Values.Data;
            
            try phi_uf = logs.getElement('phi_uf').Values.Data; catch, phi_uf = zeros(size(time)); end
            try phi_ur = logs.getElement('phi_ur').Values.Data; catch, phi_ur = zeros(size(time)); end
        catch
            error('Falha ao ler elementos pelo nome no logsout');
        end
        
    % Se tiver a variavel customizada 'results' (que o Simulink salva do ToWorkspace)
    elseif has_results
        states = simOut.results;
        time = simOut.tout;
        
        if size(states, 1) < size(states, 2)
            states = states';
        end
        
        % Indices baseados no modelo 5-DOF State-Space padrao
        if size(states, 2) >= 6
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
        else
            % Caso o numero de colunas nao seja esperado
            error('Matriz results nao tem as 6 colunas esperadas.');
        end
        
        phi_uf = zeros(size(time, 1), 1);
        phi_ur = zeros(size(time, 1), 1);

    % Fallback para xout
    elseif has_xout
        xout = simOut.xout;
        if isa(xout, 'Simulink.SimulationData.Dataset')
            time = xout.get(1).Values.Time;
            states = xout.get(1).Values.Data;
        else
            time = simOut.tout;
            states = xout;
        end
        
        if size(states, 1) < size(states, 2)
            states = states';
        end
        
        % Verifica se states tem as colunas necessarias
        if size(states, 2) >= 4
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
        else
            % Se falhar as colunas, cria array de zeros para nao quebrar a figura
            warning('Matriz states nao possui as colunas esperadas. Preenchendo com zeros para debug.');
            beta = zeros(size(time, 1), 1);
            r = zeros(size(time, 1), 1);
            phi = zeros(size(time, 1), 1);
            p = zeros(size(time, 1), 1);
        end
        
        phi_uf = zeros(size(time, 1), 1); 
        phi_ur = zeros(size(time, 1), 1);
    else
        warning('Nenhum dado válido encontrado no simOut para plotar.');
        return;
    end
catch ME
    warning('Falha ao extrair dados do simOut para plotagem: %s', ME.message);
    return;
end

%% Criar figura
fig = figure('Name', sprintf('%s - %s', metadata.controller, metadata.maneuver), ...
    'Position', [50 50 1400 900], ...
    'Color', 'w');

%% Subplot 1: Roll angle (sprung mass)
subplot(3,2,1);
plot(time, rad2deg(phi), 'LineWidth', 2.5, 'Color', [0 0.4470 0.7410]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Roll Angle φ (deg)', 'FontSize', 11);
title(sprintf('%s Controller - Roll Response', metadata.controller), 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 2: Yaw rate
subplot(3,2,2);
plot(time, rad2deg(r), 'LineWidth', 2.5, 'Color', [0.8500 0.3250 0.0980]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Rate r (deg/s)', 'FontSize', 11);
title('Yaw Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 3: Unsprung roll angles
subplot(3,2,3);
plot(time, rad2deg(phi_uf), 'LineWidth', 2, 'DisplayName', 'Front');
hold on;
plot(time, rad2deg(phi_ur), 'LineWidth', 2, 'DisplayName', 'Rear');
hold off;
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Unsprung Roll (deg)', 'FontSize', 11);
title('Unsprung Mass Roll', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
xlim([0 max(time)]);

%% Subplot 4: Sideslip angle
subplot(3,2,4);
plot(time, rad2deg(beta), 'LineWidth', 2.5, 'Color', [0.9290 0.6940 0.1250]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Sideslip β (deg)', 'FontSize', 11);
title('Lateral Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 5: Roll rate
subplot(3,2,5);
plot(time, rad2deg(p), 'LineWidth', 2.5, 'Color', [0.4940 0.1840 0.5560]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Roll Rate p (deg/s)', 'FontSize', 11);
title('Roll Rate', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 6: Metrics summary (texto)
subplot(3,2,6);
axis off;

metrics_text = {
    sprintf('\\bf{Simulation Summary}');
    sprintf('');
    sprintf('Controller: %s', metadata.controller);
    sprintf('Maneuver: %s @ %d km/h', metadata.maneuver, round(metadata.velocity_kmh));
    sprintf('');
    sprintf('\\bf{Performance Metrics}');
};

if isfield(metadata, 'metrics')
    metrics_text{end+1} = sprintf('Roll RMS: %.4f deg', metadata.metrics.rms_roll_deg);
    metrics_text{end+1} = sprintf('Roll Max: %.4f deg', metadata.metrics.max_roll_deg);
end

metrics_text{end+1} = sprintf('');
metrics_text{end+1} = sprintf('\\bf{Timestamp}');
metrics_text{end+1} = sprintf('%s', datestr(metadata.timestamp, 'dd-mmm-yyyy HH:MM:SS'));

text(0.1, 0.9, metrics_text, 'FontSize', 10, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

%% Salvar figura
output_folder = 'results/5dof/figures';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

fig_filename = sprintf('%s_%s_%s_%dkmh.png', ...
    datestr(metadata.timestamp, 'yyyymmdd_HHMMSS'), ...
    metadata.controller, ...
    metadata.maneuver, ...
    round(metadata.velocity_kmh));

fig_path = fullfile(output_folder, fig_filename);
try
    exportgraphics(fig, fig_path, 'Resolution', 300);
    fprintf('   Figura salva: %s\n', fig_path);
catch
    warning('Não foi possível salvar a figura usando exportgraphics.');
end

end

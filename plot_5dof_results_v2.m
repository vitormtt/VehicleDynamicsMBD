function fig = plot_5dof_results_v2(simOut, metadata)
%PLOT_5DOF_RESULTS_V2 Visualização profissional (Publication-ready)
%
% Input: 
%   simOut   - objeto Simulink.SimulationOutput
%   metadata - struct com metadados da simulação
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Extrair dados do Dataset do Simulink (logsout ou xout)
try
    if simOut.find('logsout')
        logs = simOut.logsout;
        time = logs.get(1).Values.Time;
        % Assumindo que os nomes dos sinais estejam configurados no Simulink
        phi = logs.getElement('phi').Values.Data;
        r = logs.getElement('r').Values.Data;
        phi_uf = logs.getElement('phi_uf').Values.Data;
        phi_ur = logs.getElement('phi_ur').Values.Data;
        beta = logs.getElement('beta').Values.Data;
        p = logs.getElement('p').Values.Data;
    elseif simOut.find('xout')
        % Fallback para xout se logsout não estiver perfeitamente nomeado
        xout = simOut.xout;
        time = simOut.tout;
        % indices aproximados para 5DOF (ajuste conforme seu state-space)
        phi = xout(:,4); 
        p = xout(:,5);
        beta = xout(:,1); % ou uy/vx
        r = xout(:,2);
        phi_uf = zeros(size(time)); % Placeholder se nao logado no xout
        phi_ur = zeros(size(time));
    else
        warning('Nenhum dado (logsout ou xout) encontrado no simOut.');
        return;
    end
catch
    warning('Falha ao extrair dados do simOut para plotagem.');
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

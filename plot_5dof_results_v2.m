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

    % Extrai variavel de tempo
    if isprop(simOut, 'tout')
        time = simOut.tout;
    elseif isprop(simOut, 'timeSim')
        time = simOut.timeSim;
    else
        error('Variavel de tempo nao encontrada no simOut');
    end
    
    % Extrai manobra (steer_ts exportada pro base workspace ou logs)
    try
        steer_rad = evalin('base', 'steer_ts.Data');
        % Interpola para o mesmo array de tempo da simulação
        steer_time = evalin('base', 'steer_ts.Time');
        steer_interp = interp1(steer_time, steer_rad, time, 'linear', 'extrap');
    catch
        steer_interp = zeros(size(time)); % Placeholder
    end

    % Tenta logsout primeiro
    if has_logsout && isa(simOut.logsout, 'Simulink.SimulationData.Dataset') && simOut.logsout.numElements > 0 && ~isempty(simOut.logsout.find('phi'))
        logs = simOut.logsout;
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
        
    % Se tiver a variavel customizada 'results' (do ToWorkspace)
    elseif has_results
        states = simOut.results;
        
        if size(states, 1) < size(states, 2)
            states = states';
        end
        
        % Indices baseados no modelo 5-DOF State-Space (Gaspar)
        % 1: beta, 2: r, 3: phi, 4: p, 5: phi_uf, 6: phi_ur
        if size(states, 2) >= 6
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
            phi_uf = states(:,5);
            phi_ur = states(:,6);
        else
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
            phi_uf = zeros(size(time));
            phi_ur = zeros(size(time));
        end

    % Fallback para xout
    elseif has_xout
        xout = simOut.xout;
        if isa(xout, 'Simulink.SimulationData.Dataset')
            states = xout.get(1).Values.Data;
        else
            states = xout;
        end
        
        if size(states, 1) < size(states, 2)
            states = states';
        end
        
        if size(states, 2) >= 6
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
            phi_uf = states(:,5);
            phi_ur = states(:,6);
        elseif size(states, 2) >= 4
            beta = states(:,1);
            r = states(:,2);
            phi = states(:,3);
            p = states(:,4);
            phi_uf = zeros(size(time));
            phi_ur = zeros(size(time));
        end
    else
        warning('Nenhum dado válido encontrado no simOut para plotar.');
        return;
    end
catch ME
    warning('Falha ao extrair dados do simOut para plotagem: %s', ME.message);
    return;
end

%% Criar figura Principal (Dashboard)
fig = figure('Name', sprintf('%s - %s', metadata.controller, metadata.maneuver), ...
    'Position', [50 50 1400 900], ...
    'Color', 'w');

%% Subplot 1: Steering Maneuver
subplot(3,2,1);
plot(time, rad2deg(steer_interp), 'LineWidth', 2.5, 'Color', 'k');
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Steering Angle (deg)', 'FontSize', 11);
title('Input Maneuver', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 2: Roll angle (sprung mass)
subplot(3,2,2);
plot(time, rad2deg(phi), 'LineWidth', 2.5, 'Color', [0 0.4470 0.7410]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Roll Angle φ (deg)', 'FontSize', 11);
title(sprintf('Sprung Mass Roll (Controller: %s)', metadata.controller), 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 3: Yaw rate
subplot(3,2,3);
plot(time, rad2deg(r), 'LineWidth', 2.5, 'Color', [0.8500 0.3250 0.0980]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Rate r (deg/s)', 'FontSize', 11);
title('Yaw Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 4: Unsprung roll angles
subplot(3,2,4);
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

%% Subplot 5: Sideslip angle
subplot(3,2,5);
plot(time, rad2deg(beta), 'LineWidth', 2.5, 'Color', [0.9290 0.6940 0.1250]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Sideslip β (deg)', 'FontSize', 11);
title('Lateral Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
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

%% Salvar figura Principal
output_folder = 'results/5dof/figures';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

base_filename = sprintf('%s_%s_%s_%dkmh', ...
    datestr(metadata.timestamp, 'yyyymmdd_HHMMSS'), ...
    metadata.controller, ...
    metadata.maneuver, ...
    round(metadata.velocity_kmh));

fig_path = fullfile(output_folder, [base_filename, '_Dashboard.png']);
exportgraphics(fig, fig_path, 'Resolution', 300);
fprintf('   Dashboard salvo: %s\n', fig_path);

%% Salvar Imagens JPG Separadas (para relatórios)
% 1. Maneuver
fig_ind = figure('Visible', 'off');
plot(time, rad2deg(steer_interp), 'LineWidth', 2.5, 'Color', 'k'); grid on;
xlabel('Time (s)'); ylabel('Steering Angle (deg)'); title('Input Maneuver');
exportgraphics(fig_ind, fullfile(output_folder, [base_filename, '_Maneuver.jpg']), 'Resolution', 300);
close(fig_ind);

% 2. Sprung Mass Roll
fig_ind = figure('Visible', 'off');
plot(time, rad2deg(phi), 'LineWidth', 2.5, 'Color', [0 0.4470 0.7410]); grid on;
xlabel('Time (s)'); ylabel('Roll Angle φ (deg)'); title(sprintf('Sprung Mass Roll (%s)', metadata.controller));
exportgraphics(fig_ind, fullfile(output_folder, [base_filename, '_Roll.jpg']), 'Resolution', 300);
close(fig_ind);

% 3. Unsprung Mass Roll
fig_ind = figure('Visible', 'off');
plot(time, rad2deg(phi_uf), 'LineWidth', 2, 'DisplayName', 'Front'); hold on;
plot(time, rad2deg(phi_ur), 'LineWidth', 2, 'DisplayName', 'Rear'); hold off; grid on;
xlabel('Time (s)'); ylabel('Unsprung Roll (deg)'); title('Unsprung Mass Roll'); legend;
exportgraphics(fig_ind, fullfile(output_folder, [base_filename, '_Unsprung.jpg']), 'Resolution', 300);
close(fig_ind);

fprintf('   Graficos JPG separados gerados com sucesso.\n');

end

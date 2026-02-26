function fig = plot_5dof_results_v2(sim_data, metadata)
%%PLOT_5DOF_RESULTS_V2 Visualização profissional
    if nargin < 2
        metadata = sim_data.Metadata;
    end
    
    if isempty(sim_data.States)
        warning('Sem dados para plotar');
        return;
    end
    
    fig = figure('Name', sprintf('%s - %s', metadata.controller, metadata.maneuver), ...
        'Position', [50 50 1400 900], 'Color', 'w');
        
    tt = sim_data.States;
    time = seconds(tt.Time);
    
    %%%% Subplot 1: Roll angle
    subplot(2,3,1);
    plot(time, rad2deg(tt.phi), 'LineWidth', 2.5);
    grid on;
    xlabel('Time (s)'); ylabel('Roll Angle \\phi (deg)');
    title(sprintf('%s - Roll Response', metadata.controller));
    
    %%%% Subplot 2: Yaw rate
    subplot(2,3,2);
    plot(time, rad2deg(tt.r), 'LineWidth', 2.5);
    grid on;
    xlabel('Time (s)'); ylabel('Yaw Rate r (deg/s)');
    title('Yaw Dynamics');
    
    %%%% Subplot 3: Unsprung roll
    subplot(2,3,3);
    plot(time, rad2deg(tt.phi_uf), 'DisplayName', 'Front'); hold on;
    plot(time, rad2deg(tt.phi_ur), 'DisplayName', 'Rear'); hold off;
    grid on; legend;
    xlabel('Time (s)'); ylabel('Unsprung Roll (deg)');
    title('Unsprung Mass Roll');
    
    %%%% Subplot 4: Sideslip
    subplot(2,3,4);
    plot(time, rad2deg(tt.beta), 'LineWidth', 2.5);
    grid on;
    xlabel('Time (s)'); ylabel('Sideslip \\beta (deg)');
    title('Sideslip Angle');
    
    %%%% Subplot 5: Steering Input (Plotado em Amplitude ou Graus Reais)
    % Achar o sinal de controle (Steering) na sim_data
    % Se for o Gaspar, ele estava dando 200 graus pq era rad2deg(4).
    % Vamos tentar plotar o steer_ts original do workspace ou dos logs.
    subplot(2,3,5);
    try
        steer_log = sim_data.logsout.get('Steering_Angle_rad');
        if ~isempty(steer_log)
            % Plotando o sinal convertido corretamente para graus
            plot(steer_log.Values.Time, rad2deg(steer_log.Values.Data), 'LineWidth', 2.5, 'Color', '#D95319');
            ylabel('Steering Cmd (deg)');
        end
    catch
        % Se nao tiver logsout, plota roll rate
        plot(time, rad2deg(tt.p), 'LineWidth', 2.5);
        ylabel('Roll Rate p (deg/s)');
    end
    grid on;
    xlabel('Time (s)'); 
    title('Driver Input / Roll Rate');
    
    %%%% Subplot 6: Metrics
    subplot(2,3,6); axis off;
    text(0.1, 0.9, {
        sprintf('\\\\bf{Metrics}');
        sprintf('Roll RMS: %.4f deg', metadata.metrics.rms_roll_deg);
        sprintf('Roll Max: %.4f deg', metadata.metrics.max_roll_deg);
        sprintf('Exec: %.2f s', metadata.metrics.execution_time_s);
    }, 'FontSize', 12, 'VerticalAlignment', 'top');

    %%%% Salvar figura
    fig_dir = fullfile('results', '5dof', 'figures');
    if ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end
    
    fig_path = fullfile(fig_dir, ...
        sprintf('%s_%s_%s_Dashboard.png', datestr(now, 'yyyymmdd_HHMMSS'), ...
        metadata.controller, metadata.maneuver));
        
    exportgraphics(fig, fig_path, 'Resolution', 300);
    fprintf('   Dashboard salvo: %s\\n', fig_path);
end

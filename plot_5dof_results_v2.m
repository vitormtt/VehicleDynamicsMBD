function fig = plot_5dof_results_v2(sim_data)
%PLOT_5DOF_RESULTS_V2 Visualização profissional (Publication-ready)
%
% Input: sim_data (struct de create_simulation_structure)
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Verificar dados
if isempty(sim_data.States)
    warning('Sem dados de estado para plotar');
    return;
end

%% Criar figura
fig = figure('Name', sprintf('%s - %s', sim_data.Metadata.controller, sim_data.Metadata.maneuver), ...
    'Position', [50 50 1400 900], ...
    'Color', 'w');

%% Extrair dados
tt_states = sim_data.States;
time = seconds(tt_states.Time);

%% Subplot 1: Roll angle (sprung mass)
subplot(3,2,1);
plot(time, rad2deg(tt_states.phi), 'LineWidth', 2.5, 'Color', [0 0.4470 0.7410]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Roll Angle φ (deg)', 'FontSize', 11);
title(sprintf('%s Controller - Roll Response', sim_data.Metadata.controller), 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 2: Yaw rate
subplot(3,2,2);
plot(time, rad2deg(tt_states.r), 'LineWidth', 2.5, 'Color', [0.8500 0.3250 0.0980]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Rate r (deg/s)', 'FontSize', 11);
title('Yaw Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 3: Unsprung roll angles
subplot(3,2,3);
plot(time, rad2deg(tt_states.phi_uf), 'LineWidth', 2, 'DisplayName', 'Front');
hold on;
plot(time, rad2deg(tt_states.phi_ur), 'LineWidth', 2, 'DisplayName', 'Rear');
hold off;
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Unsprung Roll (deg)', 'FontSize', 11);
title('Unsprung Mass Roll', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
xlim([0 max(time)]);

%% Subplot 4: Sideslip angle
subplot(3,2,4);
plot(time, rad2deg(tt_states.beta), 'LineWidth', 2.5, 'Color', [0.9290 0.6940 0.1250]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Sideslip β (deg)', 'FontSize', 11);
title('Lateral Dynamics', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 max(time)]);

%% Subplot 5: Roll rate
subplot(3,2,5);
plot(time, rad2deg(tt_states.p), 'LineWidth', 2.5, 'Color', [0.4940 0.1840 0.5560]);
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
    sprintf('Controller: %s', sim_data.Metadata.controller);
    sprintf('Maneuver: %s @ %d km/h', sim_data.Metadata.maneuver, round(sim_data.Metadata.velocity_kmh));
    sprintf('');
    sprintf('\\bf{Performance Metrics}');
    sprintf('Roll RMS: %.4f deg', sim_data.Metrics.rms_roll_deg);
    sprintf('Roll Max: %.4f deg', sim_data.Metrics.max_roll_deg);
    sprintf('Exec. Time: %.2f s', sim_data.Metrics.execution_time_s);
    sprintf('');
    sprintf('\\bf{Timestamp}');
    sprintf('%s', datestr(sim_data.Metadata.timestamp, 'dd-mmm-yyyy HH:MM:SS'));
};

text(0.1, 0.9, metrics_text, 'FontSize', 10, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

%% Salvar figura
output_folder = 'results/5dof/figures';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

fig_filename = sprintf('%s_%s_%s_%dkmh.png', ...
    datestr(sim_data.Metadata.timestamp, 'yyyymmdd_HHMMSS'), ...
    sim_data.Metadata.controller, ...
    sim_data.Metadata.maneuver, ...
    round(sim_data.Metadata.velocity_kmh));

fig_path = fullfile(output_folder, fig_filename);
exportgraphics(fig, fig_path, 'Resolution', 300);
fprintf('   Figura salva: %s\n', fig_path);

end

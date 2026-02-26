%% ═══════════════════════════════════════════════════════
%% COMPARAÇÃO MULTI-CONTROLADOR (INDUSTRIAL GRADE)
%% ═══════════════════════════════════════════════════════

function comparison = compare_simulations(sim_files)
%COMPARE_SIMULATIONS Comparação estatística entre simulações
%
% Input:
%   sim_files - cell array com caminhos dos .mat
%
% Output:
%   comparison - struct com análise comparativa

n_sims = length(sim_files);
comparison = struct();
comparison.files = sim_files;

%% Carregar todas as simulações
fprintf('Carregando %d simulações...\n', n_sims);
sims = cell(n_sims, 1);
for i = 1:n_sims
    load(sim_files{i}, 'sim_data');
    sims{i} = sim_data;
    fprintf('  [%d] %s - %s\n', i, sim_data.Metadata.controller, ...
            sim_data.Metadata.maneuver);
end

%% Tabela de Métricas Comparativas
metrics_table = table();

for i = 1:n_sims
    row = struct();
    row.Controller = sims{i}.Metadata.controller;
    row.RMS_Roll_deg = sims{i}.Metrics.rms_roll_deg;
    row.Max_Roll_deg = sims{i}.Metrics.max_roll_deg;
    row.Max_NLT = sims{i}.Metrics.max_nlt;
    row.Control_Effort = sims{i}.Metrics.control_effort_Nm2s;
    row.Exec_Time_s = sims{i}.SimulationInfo.elapsed_time_s;
    
    metrics_table = [metrics_table; struct2table(row)];
end

comparison.metrics_table = metrics_table;

%% Análise Estatística (ANOVA, Kruskal-Wallis)
% Para determinar se diferenças são significativas

%% Plotar Comparação
figure('Position', [100 100 1400 800]);

% Subplot 1: Roll angle time history
subplot(2,3,1);
hold on;
for i = 1:n_sims
    tt = sims{i}.States;
    plot(tt.Time, rad2deg(tt.phi), 'DisplayName', sims{i}.Metadata.controller, 'LineWidth', 2);
end
hold off;
grid on;
xlabel('Time (s)');
ylabel('Roll Angle (deg)');
title('Roll Response Comparison');
legend('Location', 'best');

% Subplot 2: Bar chart de métricas
subplot(2,3,2);
bar(metrics_table.Max_Roll_deg);
set(gca, 'XTickLabel', metrics_table.Controller);
ylabel('Max Roll (deg)');
title('Peak Roll Angle');
grid on;

% ... (mais subplots)

comparison.figure_handle = gcf;

end

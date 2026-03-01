function compare_simulations(varargin)
%COMPARE_SIMULATIONS Compara resultados de simulações diferentes
%
% Uso:
%   compare_simulations('results/sim1.mat', 'results/sim2.mat')
%   compare_simulations() % Abre UI para selecionar arquivos
%
% Autor: Vitor Yukio

if nargin == 0
    [files, path] = uigetfile('results/*.mat', 'Selecione as simulações', 'MultiSelect', 'on');
    if isequal(files, 0)
        return;
    end
    if ischar(files)
        files = {files};
    end
    filepaths = fullfile(path, files);
else
    filepaths = varargin;
end

num_sims = length(filepaths);
data = cell(1, num_sims);
labels = cell(1, num_sims);

for i = 1:num_sims
    loaded = load(filepaths{i});
    if isfield(loaded, 'sim_data')
        data{i} = loaded.sim_data;
        labels{i} = loaded.sim_data.Metadata.controller;
    else
        warning('Arquivo não contém struct sim_data: %s', filepaths{i});
    end
end

% Remover vazios
data = data(~cellfun('isempty', data));
labels = labels(~cellfun('isempty', labels));

if isempty(data)
    error('Nenhuma simulação válida carregada.');
end

% Plot Roll Angle
figure('Name', 'Comparação - Roll Angle', 'Position', [100 100 800 500]);
hold on; grid on; box on;
colors = lines(length(data));

for i = 1:length(data)
    t = data{i}.Time;
    phi = data{i}.States.phi_deg;
    plot(t, phi, 'LineWidth', 1.5, 'Color', colors(i,:));
end

title('Vehicle Roll Angle (\phi) Comparison');
xlabel('Time [s]');
ylabel('Roll Angle [deg]');
legend(labels, 'Location', 'best');

% Plot NLT
figure('Name', 'Comparação - NLT', 'Position', [150 150 800 600]);

subplot(2,1,1); hold on; grid on;
for i = 1:length(data)
    t = data{i}.Time;
    nlt_f = data{i}.NLT.front;
    plot(t, nlt_f, 'LineWidth', 1.5, 'Color', colors(i,:));
end
title('Front Normalized Load Transfer (NLT_f)');
ylabel('NLT [-]');
legend(labels, 'Location', 'best');
ylim([-1.2 1.2]);

subplot(2,1,2); hold on; grid on;
for i = 1:length(data)
    t = data{i}.Time;
    nlt_r = data{i}.NLT.rear;
    plot(t, nlt_r, 'LineWidth', 1.5, 'Color', colors(i,:));
end
title('Rear Normalized Load Transfer (NLT_r)');
xlabel('Time [s]');
ylabel('NLT [-]');
ylim([-1.2 1.2]);

end
function results = run_experiments(maneuver, velocity_kmh)
%RUN_EXPERIMENTS Executa bateria de simulações comparando controladores
%
% Uso:
%   run_experiments('DLC', 70)
%   run_experiments('Fishhook', 80)
%
% Autor: Vitor Yukio
% Data: 13/02/2026

if nargin < 1, maneuver = 'DLC'; end
if nargin < 2, velocity_kmh = 70; end

fprintf('========================================================\n');
fprintf(' INICIANDO BATERIA DE EXPERIMENTOS\n');
fprintf(' Manobra: %s | Velocidade: %.1f km/h\n', maneuver, velocity_kmh);
fprintf('========================================================\n\n');

controllers = {'Passive', 'PID', 'MPC'};
results = cell(1, length(controllers));

for i = 1:length(controllers)
    ctrl = controllers{i};
    fprintf('>> Testando %s...\n', ctrl);
    
    try
        res = run_5dof_simulation(ctrl, maneuver, velocity_kmh);
        results{i} = res;
    catch ME
        fprintf('   ❌ Erro ao simular %s: %s\n', ctrl, ME.message);
    end
    fprintf('\n');
end

fprintf('Bateria de testes concluída!\n');

end
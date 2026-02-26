function results = run_5dof_simulation(varargin)
%RUN_5DOF_SIMULATION Script de simulação para modelo 5-DOF (VERSÃO 2.0)
%
% Arquitetura profissional com:
%   - Dataset logging (Simulink.SimulationData)
%   - MBD compliant inputs
%   - ISO 19364 compliance
%   - Rastreabilidade (Git integration)
%
% Uso:
%   run_5dof_simulation                    % Padrão: PID, DLC, 70km/h
%   run_5dof_simulation('Passive')         % Especifica controlador
%   run_5dof_simulation('PID', 'Fishhook', 80)
%
% Outputs:
%   results - simOut padronizado
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 26/02/2026
% Versão: 2.1 (MBD Compliant)

%% ═══════════════════════════════════════════════════════
%% PARSER DE ARGUMENTOS
%% ═══════════════════════════════════════════════════════
p = inputParser;
addOptional(p, 'controller', 'PID', @(x) ismember(x, {'Passive', 'PID', 'MPC'}));
addOptional(p, 'maneuver', 'DLC', @(x) ismember(x, {'DLC', 'Fishhook', 'J-Turn', 'StepSteer'}));
addOptional(p, 'velocity_kmh', 70, @(x) x>0 && x<200);
parse(p, varargin{:});

controller = p.Results.controller;
maneuver = p.Results.maneuver;
velocity_kmh = p.Results.velocity_kmh;

%% ═══════════════════════════════════════════════════════
%% HEADER
%% ═══════════════════════════════════════════════════════
fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
fprintf('║        5-DOF VEHICLE DYNAMICS SIMULATION v2.1        ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n');
fprintf('  Vehicle:     Chevrolet Blazer 2001\n');
fprintf('  Model:       model_5dof.slx\n');
fprintf('  Controller:  %s\n');
fprintf('  Maneuver:    %s\n', maneuver);
fprintf('  Velocity:    %.1f km/h\n', velocity_kmh);
fprintf('───────────────────────────────────────────────────────\n\n');

%% ═══════════════════════════════════════════════════════
%% 1. INICIALIZAR AMBIENTE E DATA DICTIONARY
%% ═══════════════════════════════════════════════════════
fprintf('1️⃣  Inicializando Ambiente e Data Dictionary...\n');
setup_environment(); % Padroniza caminhos e cache
init_dd();
fprintf('   ✅ DD carregado\n\n');

%% ═══════════════════════════════════════════════════════
%% 2. CONFIGURAR MODO DE CONTROLE
%% ═══════════════════════════════════════════════════════
fprintf('2️⃣  Configurando controlador: %s\n', controller);

arb_mode_map = containers.Map({'Passive', 'PID', 'MPC'}, {1, 2, 3});
arb_mode = arb_mode_map(controller);

% Atualizar ARB_Mode no Data Dictionary
dd = Simulink.data.dictionary.open('data/parameters/vehicle_params.sldd');
dDataSect = getSection(dd, 'Design Data');
arb_entry = find(dDataSect, 'Name', 'ARB_Mode');

if ~isempty(arb_entry)
    arb_param = getValue(arb_entry);
    arb_param.Value = arb_mode;
    assignin(dDataSect, 'ARB_Mode', arb_param);
    saveChanges(dd);
    fprintf('   ARB_Mode = %d (atualizado no DD)\n', arb_mode);
end

close(dd);
fprintf('   ✅ Modo configurado\n\n');

%% ═══════════════════════════════════════════════════════
%% 3. ATUALIZAR VELOCIDADE
%% ═══════════════════════════════════════════════════════
if velocity_kmh ~= 70
    fprintf('3️⃣  Atualizando velocidade para %.1f km/h...\n', velocity_kmh);
    
    dd = Simulink.data.dictionary.open('data/parameters/vehicle_params.sldd');
    dDataSect = getSection(dd, 'Design Data');
    
    vehicle = getValue(getEntry(dDataSect, 'vehicle'));
    MAR = getValue(getEntry(dDataSect, 'MAR'));
    
    v_ms = velocity_kmh / 3.6;
    sys = build_active_state_space(vehicle, MAR, v_ms);
    
    A_ativo = getValue(getEntry(dDataSect, 'A_ativo'));
    A_ativo.Value = sys.A;
    assignin(dDataSect, 'A_ativo', A_ativo);
    
    B_ativo = getValue(getEntry(dDataSect, 'B_ativo'));
    B_ativo.Value = sys.B_total;
    assignin(dDataSect, 'B_ativo', B_ativo);
    
    saveChanges(dd);
    close(dd);
    
    fprintf('   ✅ Matrizes A/B recalculadas\n\n');
else
    fprintf('3️⃣  Usando velocidade padrão: 70 km/h\n\n');
end

%% ═══════════════════════════════════════════════════════
%% 4. GERAR MANOBRA (MBD)
%% ═══════════════════════════════════════════════════════
fprintf('4️⃣  Gerando manobra: %s\n', maneuver);

sim_time_map = containers.Map({'DLC', 'Fishhook', 'J-Turn', 'StepSteer'}, {10, 10, 8, 6});
sim_time = sim_time_map(maneuver);

% Usa a nova função unificada que retorna timeseries
steer_ts = create_maneuver_data(maneuver, sim_time, velocity_kmh);

% Envia para o workspace base (Onde o bloco From Workspace do Simulink vai ler)
assignin('base', 'steer_ts', steer_ts);

fprintf('   Duração: %.1f s (%d pontos)\n', sim_time, length(steer_ts.Time));
fprintf('   ✅ Sinal de esterçamento gerado e exportado como timeseries\n\n');

%% ═══════════════════════════════════════════════════════
%% 5. EXECUTAR SIMULAÇÃO (Usando SimulationInput)
%% ═══════════════════════════════════════════════════════
fprintf('5️⃣  Executando simulação...\n');

model = 'model_5dof';
model_path = fullfile('models', 'variants', '5dof', [model, '.slx']);

if ~exist(model_path, 'file')
    error('Modelo não encontrado: %s', model_path);
end

% Carregar modelo
if ~bdIsLoaded(model)
    load_system(model_path);
    fprintf('   Modelo carregado\n');
end

%% Configurar State-Space com matrizes numéricas
fprintf('   Configurando State-Space...\n');

dd = Simulink.data.dictionary.open('data/parameters/vehicle_params.sldd');
dDataSect = getSection(dd, 'Design Data');

A_mat = getValue(find(dDataSect, 'Name', 'A_ativo')).Value;
B_mat = getValue(find(dDataSect, 'Name', 'B_ativo')).Value;
C_mat = getValue(find(dDataSect, 'Name', 'C_ativo')).Value;
D_mat = getValue(find(dDataSect, 'Name', 'D_ativo')).Value;

close(dd);

ss_block = find_system(model, 'FollowLinks', 'on', 'LookUnderMasks', 'all', ...
                       'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'StateSpace');
if ~isempty(ss_block)
    set_param(ss_block{1}, 'A', mat2str(A_mat, 15));
    set_param(ss_block{1}, 'B', mat2str(B_mat, 15));
    set_param(ss_block{1}, 'C', mat2str(C_mat, 15));
    set_param(ss_block{1}, 'D', mat2str(D_mat, 15));
    set_param(ss_block{1}, 'X0', 'zeros(6,1)');
    fprintf('   ✅ State-Space configurado\n');
end

%% Configurar Simulation Input Object (MBD Standard)
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter('StopTime', num2str(sim_time));
simIn = simIn.setModelParameter('SolverType', 'Variable-step');
simIn = simIn.setModelParameter('Solver', 'ode45');
simIn = simIn.setModelParameter('RelTol', '1e-4');
simIn = simIn.setModelParameter('SaveOutput', 'on');
simIn = simIn.setModelParameter('SaveState', 'on');
simIn = simIn.setModelParameter('SaveTime', 'on');
simIn = simIn.setModelParameter('ReturnWorkspaceOutputs', 'on');

%% Executar simulação
tic;
results = sim(simIn);
elapsed = toc;

fprintf('   Tempo de execução: %.2f s\n', elapsed);
save_system(model);
fprintf('   ✅ Simulação concluída\n\n');

%% ═══════════════════════════════════════════════════════
%% 6. PÓS-PROCESSAR RESULTADOS E SALVAR
%% ═══════════════════════════════════════════════════════
fprintf('6️⃣  Processando resultados...\n');

% Calcular métricas
try
    metrics = calculate_performance_metrics_v2(results, controller);
    fprintf('   RMS Roll Angle:    %.4f deg\n', metrics.rms_roll_deg);
    fprintf('   Max Roll Angle:    %.4f deg\n', metrics.max_roll_deg);
    fprintf('   Max NLT (front):   %.3f\n', metrics.max_nlt_f);
    fprintf('   Max NLT (rear):    %.3f\n', metrics.max_nlt_r);
    fprintf('   ✅ Métricas calculadas\n\n');
catch
    fprintf('   ⚠️  Não foi possivel calcular metricas (verifique formato do simOut)\n\n');
end

filepath = save_simulation_results(results);
fprintf('   ✅ Resultados salvos em %s\n\n', filepath);

try
    plot_5dof_results_v2(results);
    fprintf('   ✅ Gráficos gerados\n\n');
catch
    fprintf('   ⚠️  Não foi possivel gerar os graficos\n\n');
end

%% ═══════════════════════════════════════════════════════
%% SUMMARY
%% ═══════════════════════════════════════════════════════
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║              SIMULATION COMPLETED ✅                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

end

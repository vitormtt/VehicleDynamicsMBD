function results = run_5dof_simulation(varargin)
%RUN_5DOF_SIMULATION Script de simulação para modelo 5-DOF (VERSÃO 2.0)
%
% Arquitetura profissional com:
%   - Dataset logging (Simulink.SimulationData)
%   - Timetable structures (metadados automáticos)
%   - ISO 19364 compliance
%   - Rastreabilidade (Git integration)
%
% Uso:
%   run_5dof_simulation                    % Padrão: PID, DLC, 70km/h
%   run_5dof_simulation('Passive')         % Especifica controlador
%   run_5dof_simulation('PID', 'Fishhook', 80)
%
% Outputs:
%   results - struct hierárquico padronizado (sim_data)
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026
% Versão: 2.0

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
fprintf('║        5-DOF VEHICLE DYNAMICS SIMULATION v2.0        ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n');
fprintf('  Vehicle:     Chevrolet Blazer 2001\n');
fprintf('  Model:       model_5dof.slx\n');
fprintf('  Controller:  %s\n', controller);
fprintf('  Maneuver:    %s\n', maneuver);
fprintf('  Velocity:    %.1f km/h\n', velocity_kmh);
fprintf('───────────────────────────────────────────────────────\n\n');

%% ═══════════════════════════════════════════════════════
%% 1. INICIALIZAR DATA DICTIONARY
%% ═══════════════════════════════════════════════════════
fprintf('1️⃣  Inicializando Data Dictionary...\n');
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
%% 3. ATUALIZAR VELOCIDADE (se diferente de 70 km/h)
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
%% 4. GERAR MANOBRA
%% ═══════════════════════════════════════════════════════
fprintf('4️⃣  Gerando manobra: %s\n', maneuver);

switch maneuver
    case 'DLC'
        steering_signal = generate_dlc_maneuver(1, 2, 3.5, 5, 10);
        sim_time = 10;
    case 'Fishhook'
        steering_signal = generate_fishhook_maneuver();
        sim_time = 10;
    case 'J-Turn'
        steering_signal = generate_j_turn_maneuver();
        sim_time = 8;
    case 'StepSteer'
        steering_signal = generate_step_steer_maneuver();
        sim_time = 6;
    otherwise
        error('Manobra não implementada: %s', maneuver);
end

assignin('base', 'steering_signal', steering_signal);
fprintf('   Duração: %.1f s (%d pontos)\n', sim_time, size(steering_signal, 1));
fprintf('   ✅ Sinal de esterçamento gerado\n\n');

%% ═══════════════════════════════════════════════════════
%% 5. EXECUTAR SIMULAÇÃO
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

%% Configurar logging
set_param(model, 'SaveOutput', 'on');
set_param(model, 'OutputSaveName', 'yout');
set_param(model, 'SaveState', 'on');
set_param(model, 'StateSaveName', 'xout');
set_param(model, 'SaveTime', 'on');
set_param(model, 'TimeSaveName', 'tout');

%% Executar simulação
tic;
simOut = sim(model, ...
    'StopTime', num2str(sim_time), ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'RelTol', '1e-4', ...
    'SaveOutput', 'on', ...
    'SaveState', 'on');

elapsed = toc;
fprintf('   Tempo de execução: %.2f s\n', elapsed);

% Salvar modelo sem fechar
save_system(model);
fprintf('   Modelo salvo (mantido aberto)\n');
fprintf('   ✅ Simulação concluída\n\n');

%% ═══════════════════════════════════════════════════════
%% 6. PÓS-PROCESSAR RESULTADOS
%% ═══════════════════════════════════════════════════════
fprintf('6️⃣  Processando resultados...\n');

% Calcular métricas
metrics = calculate_performance_metrics_v2(simOut, controller);

fprintf('   RMS Roll Angle:    %.4f deg\n', metrics.rms_roll_deg);
fprintf('   Max Roll Angle:    %.4f deg\n', metrics.max_roll_deg);
fprintf('   Max NLT (front):   %.3f\n', metrics.max_nlt_f);
fprintf('   Max NLT (rear):    %.3f\n', metrics.max_nlt_r);
fprintf('   Control Effort:    %.2f Nm²s\n', metrics.control_effort_Nm2s);
fprintf('   ✅ Métricas calculadas\n\n');

%% ═══════════════════════════════════════════════════════
%% 7. CRIAR ESTRUTURA PADRONIZADA
%% ═══════════════════════════════════════════════════════
fprintf('7️⃣  Criando estrutura de dados...\n');

% Configuração para metadata
config = struct();
config.vehicle_name = 'Chevrolet Blazer 2001';
config.controller = controller;
config.maneuver = maneuver;
config.velocity = velocity_kmh;
config.solver = 'ode45';
config.reltol = 1e-4;
config.steering_signal = steering_signal;
config.controls = []; % Será preenchido se houver dados

% Criar estrutura padronizada
results = create_simulation_structure(simOut, config, metrics);

fprintf('   ✅ Estrutura criada\n\n');

%% ═══════════════════════════════════════════════════════
%% 8. SALVAR RESULTADOS
%% ═══════════════════════════════════════════════════════
fprintf('8️⃣  Salvando resultados...\n');
filepath = save_simulation_results(results);
fprintf('   ✅ Resultados salvos\n\n');

%% ═══════════════════════════════════════════════════════
%% 9. VISUALIZAR
%% ═══════════════════════════════════════════════════════
fprintf('9️⃣  Gerando visualizações...\n');
plot_5dof_results_v2(results);
fprintf('   ✅ Gráficos gerados\n\n');

%% ═══════════════════════════════════════════════════════
%% SUMMARY
%% ═══════════════════════════════════════════════════════
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║              SIMULATION COMPLETED ✅                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n');
fprintf('  File: %s\n', filepath);
fprintf('\n');

end

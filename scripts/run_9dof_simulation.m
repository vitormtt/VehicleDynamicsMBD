function results = run_9dof_simulation(varargin)
%RUN_9DOF_SIMULATION Script de simulação para modelo 9-DOF Não-Linear
%
% Este script injeta os sinais (delta e vx) diretamente nos root Inports do 
% modelo vehicle_9dof_model.slx e executa a simulação via SimulationInput.
%
% Uso:
%   run_9dof_simulation                    % Padrão: DLC, 70km/h
%   run_9dof_simulation('Fishhook', 80)
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 01/03/2026

%% 1. Parser de Argumentos
p = inputParser;
addOptional(p, 'maneuver', 'DLC', @(x) ismember(x, {'DLC', 'Fishhook', 'J-Turn', 'StepSteer', 'Gaspar'}));
addOptional(p, 'velocity_kmh', 70, @(x) x>0 && x<200);
parse(p, varargin{:});

maneuver = p.Results.maneuver;
velocity_kmh = p.Results.velocity_kmh;
v_ms = velocity_kmh / 3.6;

fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
fprintf('║        9-DOF NON-LINEAR SIMULATION (PACEJKA)         ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n');
fprintf('  Maneuver:    %s\n', maneuver);
fprintf('  Velocity:    %.1f km/h (%.2f m/s)\n', velocity_kmh, v_ms);
fprintf('───────────────────────────────────────────────────────\n\n');

%% 2. Preparar Modelo e Dicionário
model = 'vehicle_9dof_model';
model_path = fullfile('models', 'variants', '9dof', [model, '.slx']);

if ~exist(model_path, 'file')
    error('Modelo 9-DOF não encontrado: %s', model_path);
end
if ~bdIsLoaded(model)
    load_system(model_path);
end

% Garante que os Data Dictionaries estão carregados
init_9dof_buses();
add_9dof_params_to_dd();

%% 3. Gerar Sinais de Entrada (Manobra)
sim_time_map = containers.Map({'DLC', 'Fishhook', 'J-Turn', 'StepSteer', 'Gaspar'}, {10, 10, 8, 6, 8});
sim_time = sim_time_map(maneuver);

% Usa sua função do cenário para criar a timeseries de esterçamento
steer_ts = create_maneuver_data(maneuver, sim_time, velocity_kmh);

% Cria timeseries para a velocidade longitudinal constante (vx)
t_vec = steer_ts.Time;
vx_data = ones(size(t_vec)) * v_ms;
vx_ts = timeseries(vx_data, t_vec, 'Name', 'vx');

% Agrupa as entradas em um Dataset (Formato MBD para Root Inports)
% A ordem no Dataset deve bater com os Inports (1: delta, 2: vx)
in_ds = Simulink.SimulationData.Dataset;
in_ds = in_ds.addElement(steer_ts, 'delta_rad');
in_ds = in_ds.addElement(vx_ts, 'v_x');

%% 4. Configurar e Executar Simulação
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter('StopTime', num2str(sim_time));
simIn = simIn.setModelParameter('SolverType', 'Variable-step');
simIn = simIn.setModelParameter('Solver', 'ode45');
simIn = simIn.setModelParameter('RelTol', '1e-4');

% Liga o Dataset externo aos Root Inports do modelo
simIn = simIn.setExternalInput(in_ds);

fprintf('🚀 Executando simulação (%s) com ode45...\n', model);
tic;
try
    results = sim(simIn);
    elapsed = toc;
    fprintf('✅ Simulação concluída com sucesso em %.2f s!\n\n', elapsed);
catch ME
    fprintf('❌ ERRO NA SIMULAÇÃO:\n%s\n\n', ME.message);
    rethrow(ME);
end

end
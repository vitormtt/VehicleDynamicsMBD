function sim_data = create_simulation_structure(simOut, config, metrics)
%CREATE_SIMULATION_STRUCTURE Estrutura padronizada de dados (ISO 19364)
%
% Inputs:
%   simOut  - Simulink.SimulationOutput
%   config  - struct com configuração da simulação
%   metrics - struct com métricas calculadas
%
% Output:
%   sim_data - struct hierárquico padronizado
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026
% Versão: 2.0

%% ═══════════════════════════════════════════════════════
%% METADATA
%% ═══════════════════════════════════════════════════════
sim_data.Metadata = struct();
sim_data.Metadata.timestamp = datetime('now', 'TimeZone', 'America/Sao_Paulo');
sim_data.Metadata.model_name = 'model_5dof';
sim_data.Metadata.model_version = '2.0';
sim_data.Metadata.framework = 'MBD';
sim_data.Metadata.vehicle = config.vehicle_name;
sim_data.Metadata.controller = config.controller;
sim_data.Metadata.maneuver = config.maneuver;
sim_data.Metadata.velocity_kmh = config.velocity;
sim_data.Metadata.solver = config.solver;
sim_data.Metadata.RelTol = config.reltol;
sim_data.Metadata.matlab_version = version;

% Git commit (rastreabilidade)
try
    [status, git_hash] = system('git rev-parse --short HEAD');
    if status == 0
        sim_data.Metadata.git_commit = strtrim(git_hash);
    else
        sim_data.Metadata.git_commit = 'unknown';
    end
catch
    sim_data.Metadata.git_commit = 'unknown';
end

%% ═══════════════════════════════════════════════════════
%% INPUTS
%% ═══════════════════════════════════════════════════════
if isfield(config, 'steering_signal') && ~isempty(config.steering_signal)
    sim_data.Inputs = create_inputs_timetable(simOut.tout, config.steering_signal);
else
    sim_data.Inputs = [];
end

%% ═══════════════════════════════════════════════════════
%% STATES (prioridade: xout > logsout > yout)
%% ═══════════════════════════════════════════════════════
states_extracted = false;

% Método 1: xout (estados internos do State-Space)
if isprop(simOut, 'xout') && ~isempty(simOut.xout)
    sim_data.States = create_states_timetable(simOut.tout, simOut.xout);
    states_extracted = true;
    fprintf('   Estados extraídos: xout [%s]\n', mat2str(size(simOut.xout)));
end

% Método 2: logsout (Dataset logging)
if ~states_extracted && isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
    try
        sim_data.States = extract_states_from_dataset(simOut.logsout);
        states_extracted = true;
        fprintf('   Estados extraídos: logsout (Dataset)\n');
    catch ME
        warning('Falha ao extrair de logsout: %s', ME.message);
    end
end

% Método 3: yout (outputs do modelo)
if ~states_extracted && isprop(simOut, 'yout') && ~isempty(simOut.yout)
    if isa(simOut.yout, 'timeseries')
        data = simOut.yout.Data;
    elseif istimetable(simOut.yout)
        sim_data.States = simOut.yout;
        states_extracted = true;
    else
        data = simOut.yout;
    end
    
    if ~states_extracted && exist('data', 'var')
        sim_data.States = create_states_timetable(simOut.tout, data);
        states_extracted = true;
        fprintf('   Estados extraídos: yout [%s]\n', mat2str(size(data)));
    end
end

% Fallback
if ~states_extracted
    warning('Nenhuma variável de estado encontrada. Campos disponíveis:');
    disp(fieldnames(simOut));
    sim_data.States = [];
end

%% ═══════════════════════════════════════════════════════
%% OUTPUTS (NLT, Fy, ay, etc.)
%% ═══════════════════════════════════════════════════════
if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
    try
        sim_data.Outputs = extract_outputs_from_dataset(simOut.logsout);
    catch
        sim_data.Outputs = [];
    end
else
    sim_data.Outputs = [];
end

%% ═══════════════════════════════════════════════════════
%% CONTROLS (torques ARB)
%% ═══════════════════════════════════════════════════════
if isfield(config, 'controls') && ~isempty(config.controls)
    sim_data.Controls = create_controls_timetable(simOut.tout, config.controls);
else
    % Passive não tem controle ativo
    n_samples = length(simOut.tout);
    sim_data.Controls = create_controls_timetable(simOut.tout, zeros(n_samples, 2));
end

%% ═══════════════════════════════════════════════════════
%% METRICS (ISO 19364 + Custom)
%% ═══════════════════════════════════════════════════════
sim_data.Metrics = metrics;

%% ═══════════════════════════════════════════════════════
%% SIMULATION INFO
%% ═══════════════════════════════════════════════════════
sim_data.SimulationInfo = struct();
if isprop(simOut, 'ExecutionInfo')
    sim_data.SimulationInfo.elapsed_time_s = simOut.ExecutionInfo.ExecutionTime;
else
    sim_data.SimulationInfo.elapsed_time_s = NaN;
end
sim_data.SimulationInfo.num_steps = length(simOut.tout);
sim_data.SimulationInfo.warnings = {};

% Log de warnings/erros
if isprop(simOut, 'ErrorMessage') && ~isempty(simOut.ErrorMessage)
    sim_data.SimulationInfo.warnings{end+1} = simOut.ErrorMessage;
end

end

%% ═══════════════════════════════════════════════════════
%% FUNÇÕES AUXILIARES
%% ═══════════════════════════════════════════════════════

function states = extract_states_from_dataset(logsout)
% Extrair estados de Simulink.SimulationData.Dataset
states_signal = logsout.get('states');
if ~isempty(states_signal)
    states = create_states_timetable(states_signal.Values.Time, states_signal.Values.Data);
else
    error('Sinal "states" não encontrado no Dataset');
end
end

function outputs = extract_outputs_from_dataset(logsout)
% Extrair outputs de Dataset (NLT, Fy, ay, etc.)
outputs = struct();

% Tentar extrair cada sinal
signal_names = {'NLT', 'Fy', 'ay'};
for i = 1:length(signal_names)
    sig = logsout.get(signal_names{i});
    if ~isempty(sig)
        outputs.(signal_names{i}) = sig.Values;
    end
end
end

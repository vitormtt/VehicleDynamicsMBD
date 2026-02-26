function metrics = calculate_performance_metrics_v2(simOut, controller)
%CALCULATE_PERFORMANCE_METRICS_V2 Métricas ISO 19364 + veiculares (v2.2 FINAL)
%
% Inputs:
%   simOut - Simulink.SimulationOutput ou struct
%   controller - String ('Passive', 'PID', 'MPC')
%
% Output:
%   metrics - struct com métricas padronizadas
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026 | Versão: 2.2

metrics = struct();

%% 1. DETECTAR ESTRUTURA DE simOut
fprintf('   Detectando estrutura de simOut...\n');

states = [];
time = [];

if isa(simOut, 'Simulink.SimulationOutput')
    fprintf('   Tipo: Simulink.SimulationOutput\n');
    
    % Tentar xout primeiro (estados do sistema)
    if isprop(simOut, 'xout') && ~isempty(simOut.xout)
        fprintf('   Fonte: xout\n');
        states = extract_from_xout(simOut.xout);
    % Tentar yout
    elseif isprop(simOut, 'yout') && ~isempty(simOut.yout)
        fprintf('   Fonte: yout\n');
        states = extract_from_yout(simOut.yout);
    % Tentar logsout
    elseif isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
        fprintf('   Fonte: logsout\n');
        states = extract_from_logsout(simOut.logsout);
    else
        error('Nenhuma fonte de dados encontrada em simOut');
    end
    
    % Extrair tempo
    if isprop(simOut, 'tout')
        time = simOut.tout;
    else
        error('Campo tout não encontrado');
    end
    
elseif isstruct(simOut)
    fprintf('   Tipo: struct\n');
    
    if isfield(simOut, 'xout')
        states = extract_from_xout(simOut.xout);
    elseif isfield(simOut, 'yout')
        states = extract_from_yout(simOut.yout);
    else
        error('Campos yout/xout não encontrados no struct');
    end
    
    time = simOut.tout;
else
    error('Tipo de simOut não reconhecido: %s', class(simOut));
end

%% 2. VALIDAR E NORMALIZAR ESTADOS
fprintf('   Estados extraídos: [%d x %d]\n', size(states, 1), size(states, 2));

% Garantir orientação [N x M] (N samples, M states)
if size(states, 1) < size(states, 2)
    states = states';
    fprintf('   ⚠️  Estados transpostos para [%d x %d]\n', size(states, 1), size(states, 2));
end

% Validar número mínimo de estados
if size(states, 2) < 3
    error('Insuficientes estados: esperado >= 6, recebido %d', size(states, 2));
end

% Preencher com zeros se necessário
if size(states, 2) < 6
    fprintf('   ⚠️  Preenchendo com zeros: %d → 6 estados\n', size(states, 2));
    states = [states, zeros(size(states, 1), 6 - size(states, 2))];
end

%% 3. MAPEAR ESTADOS (5-DOF: vy, r, phi, phi_dot, z_s, z_s_dot)
vy = states(:, 1);          % Lateral velocity [m/s]
r = states(:, 2);           % Yaw rate [rad/s]
phi = states(:, 3);         % Roll angle [rad]
phi_dot = states(:, 4);     % Roll rate [rad/s]

%% 4. CALCULAR MÉTRICAS DE ROLL
metrics.rms_roll_deg = rad2deg(rms(phi));
metrics.max_roll_deg = rad2deg(max(abs(phi)));
metrics.rms_roll_rad = rms(phi);
metrics.max_roll_rad = max(abs(phi));
metrics.max_roll_rate_deg_s = rad2deg(max(abs(phi_dot)));

%% 5. MÉTRICAS DE YAW
metrics.rms_yaw_rate_deg_s = rad2deg(rms(r));
metrics.max_yaw_rate_deg_s = rad2deg(max(abs(r)));

%% 6. MÉTRICAS LATERAIS
metrics.rms_lateral_vel_m_s = rms(vy);
metrics.max_lateral_vel_m_s = max(abs(vy));

% Calcular aceleração lateral (derivada numérica)
if length(time) > 1
    ay = gradient(vy, time);  % [m/s²]
    metrics.max_lateral_accel_m_s2 = max(abs(ay));
    metrics.max_lateral_accel_g = max(abs(ay)) / 9.81;
else
    metrics.max_lateral_accel_m_s2 = 0;
    metrics.max_lateral_accel_g = 0;
end

%% 7. NLT (Normalized Load Transfer) - PLACEHOLDER
metrics.max_nlt_f = 0.0;
metrics.max_nlt_r = 0.0;
metrics.nlt_note = '5-DOF model: vertical loads not computed';

%% 8. CONTROL EFFORT
if strcmp(controller, 'Passive')
    metrics.control_effort_Nm2s = 0.0;
else
    try
        if evalin('base', 'exist(''T_m'', ''var'')')
            T_m = evalin('base', 'T_m');
            if isnumeric(T_m) && length(T_m) == length(time)
                metrics.control_effort_Nm2s = trapz(time, T_m.^2);
            else
                metrics.control_effort_Nm2s = 0.0;
            end
        elseif isa(simOut, 'Simulink.SimulationOutput') && isprop(simOut, 'logsout')
            T_m_signal = find_signal(simOut.logsout, 'T_m');
            if ~isempty(T_m_signal)
                T_m = T_m_signal.Values.Data;
                metrics.control_effort_Nm2s = trapz(time, T_m.^2);
            else
                metrics.control_effort_Nm2s = 0.0;
            end
        else
            metrics.control_effort_Nm2s = 0.0;
        end
    catch
        metrics.control_effort_Nm2s = 0.0;
    end
end

%% 9. METADADOS
metrics.controller = controller;
metrics.timestamp = datetime('now');
metrics.num_samples = length(time);
metrics.duration_s = time(end);
metrics.iso_19364_ready = true;

fprintf('   ✅ Métricas calculadas com sucesso\n');

end

%% ═══════════════════════════════════════════════════════
%% FUNÇÕES AUXILIARES
%% ═══════════════════════════════════════════════════════

function states = extract_from_yout(yout)
    if isa(yout, 'Simulink.SimulationData.Dataset')
        if yout.numElements > 0
            element = yout{1};
            
            if isa(element, 'Simulink.SimulationData.Signal')
                states = element.Values.Data;
            elseif isa(element, 'Simulink.SimulationData.State')
                states = element.Values.Data;
            elseif isa(element, 'timeseries')
                states = element.Data;
            else
                try
                    states = element.Values.Data;
                catch
                    states = element.Data;
                end
            end
        else
            error('Dataset yout está vazio');
        end
    elseif isa(yout, 'timeseries')
        states = yout.Data;
    elseif isnumeric(yout)
        states = yout;
    else
        error('Tipo yout não suportado: %s', class(yout));
    end
end

function states = extract_from_xout(xout)
    if isa(xout, 'Simulink.SimulationData.Dataset')
        if xout.numElements == 0
            error('Dataset xout está vazio');
        end
        
        % CORREÇÃO: Procurar elemento com maior número de colunas
        max_cols = 0;
        best_element_idx = 1;
        
        for i = 1:xout.numElements
            element = xout{i};
            
            % Extrair dados temporariamente
            if isa(element, 'Simulink.SimulationData.State')
                temp_data = element.Values.Data;
            elseif isa(element, 'Simulink.SimulationData.Signal')
                temp_data = element.Values.Data;
            elseif isa(element, 'timeseries')
                temp_data = element.Data;
            else
                continue;
            end
            
            % Verificar dimensões
            n_cols = size(temp_data, 2);
            if n_cols > max_cols
                max_cols = n_cols;
                best_element_idx = i;
            end
        end
        
        fprintf('   → Selecionado elemento %d/%d (dimensão: ? x %d)\n', ...
                best_element_idx, xout.numElements, max_cols);
        
        % Extrair elemento ótimo
        element = xout{best_element_idx};
        
        if isa(element, 'Simulink.SimulationData.State')
            states = element.Values.Data;
        elseif isa(element, 'Simulink.SimulationData.Signal')
            states = element.Values.Data;
        elseif isa(element, 'timeseries')
            states = element.Data;
        else
            try
                states = element.Values.Data;
            catch
                error('Tipo de elemento não reconhecido: %s', class(element));
            end
        end
        
    elseif isnumeric(xout)
        states = xout;
    else
        error('Tipo xout não suportado: %s', class(xout));
    end
end

function states = extract_from_logsout(logsout)
    signal = find_signal(logsout, 'ativo_states');
    if isempty(signal)
        signal = find_signal(logsout, 'states');
    end
    if isempty(signal)
        signal = logsout{1};
    end
    
    if isa(signal, 'Simulink.SimulationData.Signal')
        states = signal.Values.Data;
    elseif isa(signal, 'Simulink.SimulationData.State')
        states = signal.Values.Data;
    elseif isa(signal, 'timeseries')
        states = signal.Data;
    else
        try
            states = signal.Values.Data;
        catch
            states = signal.Data;
        end
    end
end

function signal = find_signal(dataset, name)
    signal = [];
    if isa(dataset, 'Simulink.SimulationData.Dataset')
        try
            signal = dataset.get(name);
        catch
            % Sinal não encontrado
        end
    end
end

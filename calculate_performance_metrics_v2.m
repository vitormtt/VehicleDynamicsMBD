function metrics = calculate_performance_metrics_v2(simOut, controller)
%CALCULATE_PERFORMANCE_METRICS_V2 Métricas ISO 19364 + Custom
%
% Autor: Vitor Yukio - UnB/PIBIC

states = [];

%% Extrair estados (Dataset ou matriz)
if isprop(simOut, 'xout') && ~isempty(simOut.xout)
    if isa(simOut.xout, 'Simulink.SimulationData.Dataset')
        states = extract_states_from_dataset(simOut.xout);
    else
        states = simOut.xout;
    end
end

%% Fallback: yout
if isempty(states) && isprop(simOut, 'yout') && ~isempty(simOut.yout)
    if isa(simOut.yout, 'timeseries')
        states = simOut.yout.Data;
    else
        states = simOut.yout;
    end
end

%% Se não encontrou
if isempty(states)
    warning('Estados não encontrados');
    metrics = fill_metrics_with_nan();
    return;
end

%% Garantir orientação [T×6]
if size(states, 1) == 6
    states = states';
end

%% Roll metrics
if size(states, 2) >= 3
    phi_rad = states(:,3);
    phi_deg = rad2deg(phi_rad);
    
    metrics.rms_roll_deg = rms(phi_deg);
    metrics.max_roll_deg = max(abs(phi_deg));
    metrics.mean_roll_deg = mean(abs(phi_deg));
    metrics.std_roll_deg = std(phi_deg);
else
    metrics.rms_roll_deg = NaN;
    metrics.max_roll_deg = NaN;
    metrics.mean_roll_deg = NaN;
    metrics.std_roll_deg = NaN;
end

%% Outras métricas
metrics.max_nlt_f = NaN;
metrics.max_nlt_r = NaN;
metrics.rollover_risk = false;
metrics.control_effort_Nm2s = 0;

%% Execution time
if isprop(simOut, 'ExecutionInfo')
    metrics.execution_time_s = simOut.ExecutionInfo.ExecutionTime;
else
    metrics.execution_time_s = NaN;
end

end

function metrics = fill_metrics_with_nan()
metrics.rms_roll_deg = NaN;
metrics.max_roll_deg = NaN;
metrics.mean_roll_deg = NaN;
metrics.std_roll_deg = NaN;
metrics.max_nlt_f = NaN;
metrics.max_nlt_r = NaN;
metrics.rollover_risk = false;
metrics.control_effort_Nm2s = NaN;
metrics.execution_time_s = NaN;
end

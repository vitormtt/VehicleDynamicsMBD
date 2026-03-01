function metrics = calculate_performance_metrics_v2(simOut, controller)
%CALCULATE_PERFORMANCE_METRICS_V2 Calcula métricas chave da simulação
%
% Autor: Vitor Yukio

try
    logs = simOut.logsout;
    phi = logs.get('phi').Values.Data * (180/pi);
    nlt_f = logs.get('NLT_f').Values.Data;
    nlt_r = logs.get('NLT_r').Values.Data;
    ay = logs.get('ay').Values.Data;
    
    try
        Tf = logs.get('T_f').Values.Data;
        Tr = logs.get('T_r').Values.Data;
    catch
        Tf = zeros(size(phi));
        Tr = zeros(size(phi));
    end
    
    t = logs.get('phi').Values.Time;
catch
    error('Sinais necessários não encontrados no logsout.');
end

% Cálculos
metrics.rms_roll_deg = rms(phi);
metrics.max_roll_deg = max(abs(phi));
metrics.rms_ay = rms(ay);

metrics.max_nlt_f = max(abs(nlt_f));
metrics.max_nlt_r = max(abs(nlt_r));
metrics.rollover_index = max(max(abs(nlt_f)), max(abs(nlt_r))); % RI simplificado

if strcmp(controller, 'Passive')
    metrics.energy_kj = 0;
    metrics.max_torque_nm = 0;
else
    power_f = abs(Tf .* diff([0; phi]) ./ diff([0; t]));
    power_r = abs(Tr .* diff([0; phi]) ./ diff([0; t]));
    metrics.energy_kj = trapz(t, power_f + power_r) / 1000;
    metrics.max_torque_nm = max(max(abs(Tf)), max(abs(Tr)));
end

metrics.execution_time_s = simOut.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;

end
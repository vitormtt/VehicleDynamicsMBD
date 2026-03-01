function filepath = save_simulation_results(simOut, metadata)
%SAVE_SIMULATION_RESULTS Extrai e salva resultados no formato MBD
%
% Autor: Vitor Yukio

% 1. Extrair sinais
try
    logs = simOut.logsout;
    
    t = logs.get('phi').Values.Time;
    
    % Estados
    data.Time = t;
    data.States.phi_deg   = logs.get('phi').Values.Data * (180/pi);
    data.States.theta_deg = logs.get('theta').Values.Data * (180/pi);
    data.States.psi_rate  = logs.get('psi_dot').Values.Data;
    data.States.vy        = logs.get('vy').Values.Data;
    
    % Aceleração Lateral
    data.ay = logs.get('ay').Values.Data;
    
    % NLT
    data.NLT.front = logs.get('NLT_f').Values.Data;
    data.NLT.rear  = logs.get('NLT_r').Values.Data;
    
    % Torques (se ativo)
    try
        data.Controls.Tf = logs.get('T_f').Values.Data;
        data.Controls.Tr = logs.get('T_r').Values.Data;
    catch
        data.Controls.Tf = zeros(size(t));
        data.Controls.Tr = zeros(size(t));
    end
catch
    error('Sinais não encontrados no logsout. Verifique o Signal Logging no Simulink.');
end

% 2. Estruturar
sim_data.Data = data;
sim_data.Metadata = metadata;

% 3. Salvar
results_dir = fullfile('results', '5dof', datestr(now, 'yyyy_mm'));
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

filename = sprintf('%s_%s_%skmh_%s.mat', ...
    metadata.controller, ...
    metadata.maneuver, ...
    num2str(metadata.velocity_kmh), ...
    datestr(now, 'ddHHMM'));

filepath = fullfile(results_dir, filename);
save(filepath, 'sim_data');

end
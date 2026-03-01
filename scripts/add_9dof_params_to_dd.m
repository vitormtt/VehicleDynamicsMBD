% scripts/add_9dof_params_to_dd.m
function add_9dof_params_to_dd()
    % Adiciona parâmetros de Suspensão e Pacejka MF no Data Dictionary
    
    % 1. Parâmetros da Suspensão e Pneus (Spring/Damper)
    susp_params.K_sf = 75e3;   % N/m (Front Suspension Stiffness)
    susp_params.C_sf = 5e3;    % Ns/m (Front Damping)
    susp_params.K_sr = 70e3;   % N/m (Rear Suspension Stiffness)
    susp_params.C_sr = 4e3;    % Ns/m (Rear Damping)
    susp_params.K_t  = 175.5e3; % N/m (Tire Vertical Stiffness)
    susp_params.m_uf = 80;     % kg (Front Unsprung mass - Estimado se não exato)
    susp_params.m_ur = 110;    % kg (Rear Unsprung mass - Estimado se não exato)
    
    % 2. Parâmetros Pacejka Magic Formula 5.2 (Puro Slip Lateral Fy)
    % Estrutura contendo os 8 coeficientes para cálculo de Fy
    % Nota: Valores baseados no artigo de referência (Khalil)
    pacejka_params.pCy1 = 1.3;    % Shape factor
    pacejka_params.pDy1 = 1.0;    % Peak factor
    pacejka_params.pEy1 = -0.5;   % Curvature factor
    pacejka_params.pKy1 = -20;    % Cornering stiffness parameter
    pacejka_params.pHy1 = 0;      % Horizontal shift
    pacejka_params.pVy1 = 0;      % Vertical shift
    pacejka_params.pDy2 = 0;      % Variation of D with load
    pacejka_params.pEy2 = 0;      % Variation of E with load
    
    %% Salvar no Data Dictionary
    dd_path = fullfile('data', 'parameters', 'vehicle_params.sldd');
    if exist(dd_path, 'file')
        dd = Simulink.data.dictionary.open(dd_path);
        dDataSect = getSection(dd, 'Design Data');
        
        assignin(dDataSect, 'susp_params', susp_params);
        assignin(dDataSect, 'pacejka_params', pacejka_params);
        
        saveChanges(dd);
        close(dd);
        disp('✅ Parâmetros de Suspensão e Pacejka salvos em vehicle_params.sldd');
    else
        warning('Dicionário %s não encontrado.', dd_path);
    end
end
function create_gaspar_dd()
%CREATE_GASPAR_DD Cria o Data Dictionary com os parametros de Gaspar (2004)
% Este script cria um arquivo gaspar_params.sldd que deve ser vinculado
% ao modelo Simulink para simular o caminhao do artigo.
%
% Referencia:
% Gaspar, P., et al. (2004) 'The design of a combined control structure to 
% prevent the rollover of heavy vehicles', European Journal of Control.

    dd_name = 'gaspar_params.sldd';
    
    % Se ja existir, deleta para criar do zero
    if isfile(dd_name)
        delete(dd_name);
    end
    
    % Cria o Data Dictionary
    dictObj = Simulink.data.dictionary.create(dd_name);
    sectObj = getSection(dictObj, 'Design Data');
    
    % --- Parametros Inerciais e de Massa (Tabela 2 do VU et al. / Gaspar) ---
    addEntry(sectObj, 'm_s', 12487);         % Sprung mass [kg]
    addEntry(sectObj, 'm_uf', 706);          % Unsprung mass front [kg]
    addEntry(sectObj, 'm_ur', 1000);         % Unsprung mass rear [kg]
    addEntry(sectObj, 'm', 14193);           % Total mass [kg]
    addEntry(sectObj, 'I_xx', 24201);        % Roll moment of inertia [kg*m^2]
    addEntry(sectObj, 'I_zz', 34917);        % Yaw moment of inertia [kg*m^2]
    addEntry(sectObj, 'I_xz', 4200);         % Yaw-roll product of inertia [kg*m^2]
    
    % --- Parametros Geometricos ---
    addEntry(sectObj, 'h', 1.15);            % Height of CG of sprung mass from roll axis [m]
    addEntry(sectObj, 'h_uf', 0.53);         % Height of CG of unsprung mass front [m]
    addEntry(sectObj, 'h_ur', 0.53);         % Height of CG of unsprung mass rear [m]
    addEntry(sectObj, 'h_r', 0.83);          % Height of roll axis from ground [m]
    addEntry(sectObj, 'l_f', 1.95);          % Distance front axle to CG [m]
    addEntry(sectObj, 'l_r', 1.54);          % Distance rear axle to CG [m]
    addEntry(sectObj, 'l_w', 0.93);          % Half track width [m] (track width = 1.86m)
    
    % --- Parametros de Suspensao e Pneu ---
    addEntry(sectObj, 'K_sf', 380e3);        % Front suspension roll stiffness [Nm/rad]
    addEntry(sectObj, 'K_sr', 684e3);        % Rear suspension roll stiffness [Nm/rad]
    addEntry(sectObj, 'C_sf', 100e3);        % Front suspension roll damping [Nms/rad]
    addEntry(sectObj, 'C_sr', 100e3);        % Rear suspension roll damping [Nms/rad]
    addEntry(sectObj, 'K_tf', 2060e3);       % Front tire roll stiffness [Nm/rad]
    addEntry(sectObj, 'K_tr', 3337e3);       % Rear tire roll stiffness [Nm/rad]
    addEntry(sectObj, 'C_alpha_f', 582e3);   % Front cornering stiffness [N/rad]
    addEntry(sectObj, 'C_alpha_r', 783e3);   % Rear cornering stiffness [N/rad]
    
    % --- Parametros da Barra Estabilizadora (Passive Anti-roll bar) ---
    addEntry(sectObj, 'K_arf', 10730);       % Front ARB stiffness [Nm/rad]
    addEntry(sectObj, 'K_arr', 15480);       % Rear ARB stiffness [Nm/rad]
    
    % Constante gravitacional
    addEntry(sectObj, 'g', 9.81);

    % Salva o dicionario
    saveChanges(dictObj);
    close(dictObj);
    
    fprintf('Data Dictionary %s criado com sucesso com os parametros de Gaspar (2004)!\n', dd_name);
end

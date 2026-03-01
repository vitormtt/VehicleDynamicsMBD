% scripts/init_9dof_buses.m
function init_9dof_buses()
    % Cria os Bus Objects padronizados para o modelo 9-DOF/14-DOF e salva no Dicionário
    
    %% 1. Chassis States Bus (7-DOF Ride + 2-DOF Handling)
    elems(1) = Simulink.BusElement; elems(1).Name = 'X';
    elems(2) = Simulink.BusElement; elems(2).Name = 'Y';
    elems(3) = Simulink.BusElement; elems(3).Name = 'Z';
    elems(4) = Simulink.BusElement; elems(4).Name = 'phi';   % Roll
    elems(5) = Simulink.BusElement; elems(5).Name = 'theta'; % Pitch
    elems(6) = Simulink.BusElement; elems(6).Name = 'psi';   % Yaw
    elems(7) = Simulink.BusElement; elems(7).Name = 'vx';
    elems(8) = Simulink.BusElement; elems(8).Name = 'vy';
    elems(9) = Simulink.BusElement; elems(9).Name = 'vz';
    elems(10) = Simulink.BusElement; elems(10).Name = 'p';   % Roll rate
    elems(11) = Simulink.BusElement; elems(11).Name = 'q';   % Pitch rate
    elems(12) = Simulink.BusElement; elems(12).Name = 'r';   % Yaw rate
    
    ChassisBus = Simulink.Bus;
    ChassisBus.Elements = elems;
    clear elems;

    %% 2. Tire Forces Bus (Fx, Fy, Fz para as 4 rodas)
    wheels = {'fl', 'fr', 'rl', 'rr'};
    forces = {'Fx', 'Fy', 'Fz', 'Mz'};
    k = 1;
    for i=1:4
        for j=1:4
            elems(k) = Simulink.BusElement; 
            elems(k).Name = sprintf('%s_%s', forces{j}, wheels{i});
            k = k + 1;
        end
    end
    
    TireForcesBus = Simulink.Bus;
    TireForcesBus.Elements = elems;
    clear elems;

    %% 3. Suspension Bus (Deslocamentos e forças não-suspensas)
    k = 1;
    for i=1:4
        elems(k) = Simulink.BusElement; elems(k).Name = sprintf('z_us_%s', wheels{i});
        elems(k+1) = Simulink.BusElement; elems(k+1).Name = sprintf('dz_us_%s', wheels{i});
        elems(k+2) = Simulink.BusElement; elems(k+2).Name = sprintf('Fs_%s', wheels{i}); % Força da susp.
        k = k + 3;
    end
    
    SuspensionBus = Simulink.Bus;
    SuspensionBus.Elements = elems;
    clear elems;
    
    %% Salvar no Data Dictionary
    dd_path = fullfile('data', 'parameters', 'vehicle_params.sldd');
    if exist(dd_path, 'file')
        dd = Simulink.data.dictionary.open(dd_path);
        dDataSect = getSection(dd, 'Design Data');
        
        assignin(dDataSect, 'ChassisStatesBus', ChassisBus);
        assignin(dDataSect, 'TireForcesBus', TireForcesBus);
        assignin(dDataSect, 'SuspensionBus', SuspensionBus);
        
        saveChanges(dd);
        close(dd);
        disp('✅ Bus Objects salvos em vehicle_params.sldd');
    else
        warning('Dicionário %s não encontrado.', dd_path);
    end
end
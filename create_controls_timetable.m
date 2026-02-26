function tt = create_controls_timetable(time, torques)
%CREATE_CONTROLS_TIMETABLE Timetable para torques de controle ARB
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Validação
if size(torques, 2) ~= 2
    error('Torques devem ter 2 colunas [T_front, T_rear]');
end

%% Criar timetable
tt = timetable(seconds(time), ...
    torques(:,1), torques(:,2), ...
    'VariableNames', {'T_front', 'T_rear'});

tt.Properties.VariableUnits = {'Nm', 'Nm'};
tt.Properties.VariableDescriptions = {'ARB torque front', 'ARB torque rear'};
tt.Properties.Description = 'Active Anti-Roll Bar Control Torques';

end

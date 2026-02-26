function tt = create_states_timetable(time, states)
%CREATE_STATES_TIMETABLE Criar timetable padronizado para estados
%
% Estados do modelo 5-DOF:
%   1. beta    [rad]   - Sideslip angle
%   2. r       [rad/s] - Yaw rate
%   3. phi     [rad]   - Roll angle (sprung mass)
%   4. p       [rad/s] - Roll rate
%   5. phi_uf  [rad]   - Roll angle (unsprung front)
%   6. phi_ur  [rad]   - Roll angle (unsprung rear)
%
% Autor: Vitor Yukio - UnB/PIBIC

%% Validação
if size(states, 2) ~= 6
    error('Estados devem ter 6 colunas, recebido: %d', size(states, 2));
end

if size(states, 1) ~= length(time)
    error('Dimensão incompatível: time=%d, states=%d', length(time), size(states,1));
end

%% Criar timetable
tt = timetable(seconds(time), ...
    states(:,1), states(:,2), states(:,3), ...
    states(:,4), states(:,5), states(:,6), ...
    'VariableNames', {'beta', 'r', 'phi', 'p', 'phi_uf', 'phi_ur'});

%% Metadados (autodocumentação)
tt.Properties.VariableUnits = {'rad', 'rad/s', 'rad', 'rad/s', 'rad', 'rad'};

tt.Properties.VariableDescriptions = {
    'Sideslip angle (β)', ...
    'Yaw rate (r)', ...
    'Roll angle sprung mass (φ)', ...
    'Roll rate (p)', ...
    'Roll angle unsprung front (φ_uf)', ...
    'Roll angle unsprung rear (φ_ur)'
};

tt.Properties.Description = '5-DOF Vehicle States (Yaw-Roll Model)';
tt.Properties.UserData = struct('model', '5dof', 'reference', 'Khalil2018');

end

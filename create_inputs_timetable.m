function tt = create_inputs_timetable(time, steering)
%CREATE_INPUTS_TIMETABLE Cria timetable para sinais de entrada (steering)
%
% Inputs:
%   time     - Vetor de tempo [Nx1] (s)
%   steering - Sinal de esterçamento [Nx1] ou [Nx2] (rad)
%
% Output:
%   tt - Timetable com colunas delta_f, delta_r
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026

%% Validação de inputs
if isempty(time) || isempty(steering)
    error('create_inputs_timetable:EmptyInput', 'time ou steering vazio');
end

%% Garantir que time é coluna
if isrow(time)
    time = time';
end

%% Processar steering signal
if size(steering, 2) == 2
    % Se tem 2 colunas, assumir [time, steering_angle]
    steering_data = steering(:, 2);
else
    % Se 1 coluna, usar diretamente
    steering_data = steering;
end

%% Garantir que steering_data é coluna
if isrow(steering_data)
    steering_data = steering_data';
end

%% Resolver incompatibilidade de dimensões
if length(time) ~= length(steering_data)
    fprintf('   ⚠️  Interpolando steering: %d → %d pontos\n', ...
            length(steering_data), length(time));
    
    % Criar vetor de tempo original normalizado
    time_steering = linspace(0, time(end), length(steering_data))';
    
    % Interpolar
    steering_data = interp1(time_steering, steering_data, time, 'linear', 'extrap');
end

%% Criar timetable
% Assumir steering traseiro = 0 (tração dianteira)
delta_f = steering_data;
delta_r = zeros(size(delta_f));

tt = timetable(seconds(time), delta_f, delta_r, ...
    'VariableNames', {'delta_f', 'delta_r'});

%% Metadados
tt.Properties.VariableUnits = {'rad', 'rad'};
tt.Properties.VariableDescriptions = {'Front steering angle', 'Rear steering angle'};
tt.Properties.Description = 'Vehicle steering inputs (ISO 19364 compliant)';

end

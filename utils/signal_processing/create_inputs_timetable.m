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
% Data: 13/02/2026 | Versão: 2.0

%% 1. VALIDAÇÃO DE INPUTS
if isempty(time) || isempty(steering)
    error('create_inputs_timetable:EmptyInput', 'time ou steering vazio');
end

%% 2. GARANTIR QUE TIME É COLUNA
if isrow(time)
    time = time(:);
end

%% 3. PROCESSAR STEERING SIGNAL
if size(steering, 2) == 2
    % Se tem 2 colunas, assumir [time, steering_angle]
    steering_data = steering(:, 2);
elseif size(steering, 2) == 1
    % Se 1 coluna, usar diretamente
    steering_data = steering;
else
    error('create_inputs_timetable:InvalidDimension', ...
          'steering deve ter 1 ou 2 colunas, recebido %d', size(steering, 2));
end

%% 4. GARANTIR QUE STEERING_DATA É COLUNA
if isrow(steering_data)
    steering_data = steering_data(:);
end

%% 5. RESOLVER INCOMPATIBILIDADE DE DIMENSÕES
if length(time) ~= length(steering_data)
    fprintf('   ⚠️  Interpolando steering: %d → %d pontos\n', ...
            length(steering_data), length(time));
    
    % Criar vetor de tempo original normalizado
    time_steering = linspace(0, time(end), length(steering_data))';
    
    % Interpolar (linear com extrapolação)
    steering_data = interp1(time_steering, steering_data, time, 'linear', 'extrap');
end

%% 6. CRIAR TIMETABLE
% Assumir steering traseiro = 0 (veículo tração dianteira)
delta_f = steering_data;
delta_r = zeros(size(delta_f));

% Criar timetable
tt = timetable(seconds(time), delta_f, delta_r, ...
    'VariableNames', {'delta_f', 'delta_r'});

%% 7. ADICIONAR METADADOS (ISO 19364 COMPLIANCE)
tt.Properties.VariableUnits = {'rad', 'rad'};
tt.Properties.VariableDescriptions = {'Front steering angle', 'Rear steering angle'};
tt.Properties.Description = 'Vehicle steering inputs (ISO 19364 compliant)';
tt.Properties.UserData = struct('maneuver_type', 'DLC', 'created', datetime('now'));

end

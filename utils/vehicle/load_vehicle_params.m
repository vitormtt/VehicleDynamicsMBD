function vehicle = load_vehicle_params(type)
%LOAD_VEHICLE_PARAMS Carrega parâmetros do veículo Chevrolet Blazer 2001
%
% Sintaxe:
%   vehicle = load_vehicle_params(type)
%
% Entradas:
%   type - String: 'truck' (único suportado)
%
% Saídas:
%   vehicle - Struct com 40 parâmetros físicos
%
% Referência:
%   Khalil et al. (2019), SAE Int. J. Passeng. Cars, 12(1):35-50
%   DOI: 10.4271/06-12-01-0003
%
% Exemplo:
%   truck = load_vehicle_params('truck');
%   fprintf('Massa: %.0f kg, SSF: %.2f\n', truck.m, truck.SSF);
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    switch type
        case 'truck'
            %% Massa e Inércia
            vehicle.m = 14193;      % Massa total (kg)
            vehicle.ms = 12487;     % Massa suspensa (kg)
            vehicle.muf = 706;      % Massa não suspensa frontal (kg)
            vehicle.mur = 1000;     % Massa não suspensa traseira (kg)
            vehicle.Ixx = 24201;    % Momento de inércia de rolagem (kg·m²)
            vehicle.Izz = 34917;    % Momento de inércia de guinada (kg·m²)
            vehicle.Ixz = 4200;     % Produto de inércia yaw-roll (kg·m²)
            
            %% Geometria do Veículo
            vehicle.h = 0.83;       % Altura CG massa suspensa (m)
            vehicle.hu = 0.53;      % Altura CG massa não suspensa (m)
            vehicle.rf = 1.15;      % Altura eixo de rolagem frontal (m)
            vehicle.rr = 1.15;      % Altura eixo de rolagem traseiro (m)
            vehicle.lf = 1.95;      % Distância CG → eixo dianteiro (m)
            vehicle.lr = 1.54;      % Distância CG → eixo traseiro (m)
            vehicle.lw = 0.93;      % Metade da largura do veículo (m)
            vehicle.w = vehicle.lf + vehicle.lr; % Wheelbase (m)
            
            %% Suspensão
            vehicle.kf = 380000;    % Rigidez rolagem suspensão frontal (Nm/rad)
            vehicle.kr = 684000;    % Rigidez rolagem suspensão traseira (Nm/rad)
            vehicle.bf = 100000;    % Amortecimento rolagem frontal (Nms/rad)
            vehicle.br = 100000;    % Amortecimento rolagem traseiro (Nms/rad)
            
            %% Pneus
            vehicle.ktf = 2060000;  % Rigidez rolagem pneu frontal (Nm/rad)
            vehicle.ktr = 3337000;  % Rigidez rolagem pneu traseiro (Nm/rad)
            vehicle.Cf = 582000;    % Cornering stiffness frontal (N/rad)
            vehicle.Cr = 783000;    % Cornering stiffness traseira (N/rad)
            
            %% Barra Estabilizadora (ARB) - Geometria
            vehicle.tAf = 2.4076;   % Braço principal frontal (m)
            vehicle.tBf = 0.6593;   % Braço secundário frontal (m)
            vehicle.cf = 0.8931;    % Fator geométrico frontal (m)
            vehicle.tAr = 2.8629;   % Braço principal traseiro (m)
            vehicle.tBr = 0.4113;   % Braço secundário traseiro (m)
            vehicle.cr = 0.7651;    % Fator geométrico traseiro (m)
            
            %% ARB - Rigidez Torcional
            vehicle.kAO_f = 10730;  % Rigidez ARB frontal (Nm/rad)
            vehicle.kAO_r = 15480;  % Rigidez ARB traseira (Nm/rad)
            
            %% ARB - Dimensional (informativo)
            vehicle.Df = 32;        % Diâmetro externo frontal (mm)
            vehicle.Dr = 34;        % Diâmetro externo traseiro (mm)
            
            %% Parâmetros Ambientais
            vehicle.mu = 1.0;       % Coeficiente de aderência (asfalto seco)
            vehicle.g = 9.81;       % Aceleração da gravidade (m/s²)
            
            %% Operacional
            vehicle.v = 70 / 3.6;   % Velocidade longitudinal nominal (m/s)
            
            %% Métricas de Estabilidade
            vehicle.SSF = (vehicle.lw * 2) / (2 * vehicle.h); % Static Stability Factor
            
            %% Metadata
            vehicle.name = 'Chevrolet Blazer 2001';
            vehicle.reference = 'Khalil et al. (2019), SAE 2019-01-0003';
            vehicle.dof = 14; % Modelo completo (6 sprung + 4 unsprung + 4 wheel)
            
        otherwise
            error('VehicleParams:InvalidType', ...
                'Tipo de veículo inválido: "%s". Use ''truck''.', type);
    end
    
    %% Validação de Consistência Física
    mass_check = vehicle.ms + vehicle.muf + vehicle.mur;
    assert(abs(vehicle.m - mass_check) < 1, ...
        'VehicleParams:MassInconsistency', ...
        'Inconsistência de massa: m=%.0f ≠ ms+muf+mur=%.0f', ...
        vehicle.m, mass_check);
    
    wb_check = vehicle.lf + vehicle.lr;
    assert(abs(vehicle.w - wb_check) < 0.01, ...
        'VehicleParams:WheelbaseInconsistency', ...
        'Inconsistência de wheelbase: w=%.2f ≠ lf+lr=%.2f', ...
        vehicle.w, wb_check);
end

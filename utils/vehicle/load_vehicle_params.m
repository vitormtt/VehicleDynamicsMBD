function vehicle = load_vehicle_params(type)
%LOAD_VEHICLE_PARAMS Carrega parâmetros do veículo
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
%   Gaspar et al. (2004) / Vu et al. (2016)
%   Single Unit Heavy Vehicle (Caminhão Pesado)
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    switch type
        case 'truck'
            %% Massa e Inércia
            vehicle.ms = 12487; % Massa suspensa (kg)
            vehicle.muf = 706; % Massa não suspensa no eixo dianteiro (kg)
            vehicle.mur = 1000; % Massa não suspensa no eixo traseiro (kg)
            vehicle.m = 14193; % Massa total do veículo (kg)
            
            vehicle.Ixx = 24201; % Momento de inércia de rolagem (kg.m²)
            vehicle.Ixz = 4200; % Produto de inércia yaw-roll (kg.m²)
            vehicle.Izz = 34917; % Momento de inércia de guinada (kg.m²)
            
            %% Geometria do Veículo
            vehicle.h = 0.83; % Altura do CG da massa suspensa (m)
            vehicle.hu = 0.53; % Altura do CG da massa não suspensa (m)
            
            vehicle.rf = 1.15; % Altura do eixo de rolagem ao solo (m)
            vehicle.rr = 1.15; % Altura do eixo de rolagem ao solo (m)
            
            vehicle.lf = 1.95; % Distância do CG ao eixo dianteiro (m)
            vehicle.lr = 1.54; % Distância do CG ao eixo traseiro (m)
            vehicle.w = vehicle.lf + vehicle.lr; % Wheelbase (m)
            vehicle.lw = 0.93; % Metade da largura do veículo (m)
            
            %% Suspensão
            vehicle.kf = 380000; % Rigidez de rolagem da suspensão dianteira (Nm/rad)
            vehicle.kr = 684000; % Rigidez de rolagem da suspensão traseira (Nm/rad)
            vehicle.bf = 100000; % Amortecimento de rolagem dianteiro (Nm/rad)
            vehicle.br = 100000; % Amortecimento de rolagem traseiro (Nm/rad)
            
            %% Pneus
            vehicle.ktf = 2060000; % Rigidez de rolagem do pneu dianteiro (Nm/rad)
            vehicle.ktr = 3337000; % Rigidez de rolagem do pneu traseiro (Nm/rad)
            vehicle.Cf = 582000; % Rigidez lateral do pneu dianteiro (N/rad)
            vehicle.Cr = 783000; % Rigidez lateral do pneu traseiro (N/rad)
            
            %% Barra Estabilizadora (ARB) - Geometria
            vehicle.tAf = 2.4076;
            vehicle.tBf = 0.6593;
            vehicle.cf = 0.8931;
            
            vehicle.tAr = 2.8629;
            vehicle.tBr = 0.4113;
            vehicle.cr = 0.7651;
            
            %% ARB - Rigidez Torcional
            vehicle.kAO_f = 10730; % Rigidez da Barra Dianteira (Nm/rad)
            vehicle.kAO_r = 15480; % Rigidez da Barra Traseira (Nm/rad)
            
            %% ARB - Dimensional (informativo)
            vehicle.Df = 32; % Diâmetro externo frontal (mm)
            vehicle.Dr = 34; % Diâmetro externo traseiro (mm)
            
            %% Parâmetros Ambientais
            vehicle.mu = 1.0;       % Coeficiente de aderência (asfalto seco)
            vehicle.g = 9.81;       % Aceleração da gravidade (m/s²)
            
            %% Operacional
            vehicle.v = 70 / 3.6;   % Velocidade longitudinal nominal (m/s)
            
            %% Métricas de Estabilidade
            vehicle.SSF = (vehicle.lw * 2) / (2 * vehicle.h); % Static Stability Factor
            
            %% Metadata
            vehicle.name = 'Single Unit Heavy Vehicle (Gaspar 2004 / Vu 2016)';
            vehicle.reference = 'Gaspar et al. (2004), Vu et al. (2016)';
            vehicle.dof = 14; 
            
        otherwise
            error('VehicleParams:InvalidType', ...
                'Tipo de veículo inválido: \"%s\". Use ''truck''.', type);
    end
end

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
%   Código base anterior - Blazer/Caminhonete
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    switch type
        case 'truck'
            %% Massa e Inércia
            vehicle.ms = 1760; % Massa suspensa (kg)
            vehicle.muf = 61; % Massa não suspensa no eixo dianteiro (kg)
            vehicle.mur = 53; % Massa não suspensa no eixo traseiro (kg)
            vehicle.m = vehicle.ms + vehicle.muf + vehicle.mur; % Massa total (kg)
            
            vehicle.Ixx = 527.927421; % Momento de inércia de rolagem (kg·m²)
            vehicle.Ixz = 0.059089; % Produto de inércia yaw-roll (kg·m²)
            vehicle.Izz = 2943.60885; % Momento de inércia de guinada (kg·m²)
            
            %% Geometria do Veículo
            vehicle.h = 0.626; % Altura CG massa suspensa (m)
            vehicle.hu = 0.30; % Altura CG massa não suspensa (m)
            
            vehicle.rf = 0.07; % Altura eixo de rolagem frontal (m)
            vehicle.rr = 0.19; % Altura eixo de rolagem traseiro (m)
            
            vehicle.lf = 1.332; % Distância CG → eixo dianteiro (m)
            vehicle.w = 2.874; % Wheelbase (m)
            vehicle.lr = vehicle.w - vehicle.lf; % Distância CG → eixo traseiro (m)
            
            vehicle.lw = 2.874 / 2; % Referência mantida pra retrocompatibilidade
            
            %% Suspensão
            vehicle.kf = 95760.15; % Rigidez rolagem suspensão frontal (Nm/rad)
            vehicle.kr = 53857.62; % Rigidez rolagem suspensão traseira (Nm/rad)
            vehicle.bf = 6121.10; % Amortecimento rolagem frontal (Nms/rad)
            vehicle.br = 3162.43; % Amortecimento rolagem traseiro (Nms/rad)
            
            %% Pneus
            vehicle.ktf = 258000; % Rigidez rolagem pneu frontal (Nm/rad)
            vehicle.ktr = 229000; % Rigidez rolagem pneu traseiro (Nm/rad)
            vehicle.Cf = 68800; % Cornering stiffness frontal (N/rad)
            vehicle.Cr = 103200; % Cornering stiffness traseira (N/rad)
            
            %% Barra Estabilizadora (ARB) - Geometria
            vehicle.tAf = 0.766; % Braço principal frontal (m)
            vehicle.tBf = 0.550; % Braço secundário frontal (m)
            vehicle.cf = 0.285; % Fator geométrico frontal (m)
            
            vehicle.tAr = 0.732; % Braço principal traseiro (m)
            vehicle.tBr = 0.514; % Braço secundário traseiro (m)
            vehicle.cr = 0.285; % Fator geométrico traseiro (m)
            
            %% ARB - Rigidez Torcional
            vehicle.kAO_f = 2500; % Rigidez ARB frontal (Nm/rad)
            vehicle.kAO_r = 1500; % Rigidez ARB traseira (Nm/rad)
            
            %% ARB - Dimensional (informativo)
            vehicle.Df = 2.253e-02; % Diâmetro externo frontal (mm)
            vehicle.Dr = 2.000e-02; % Diâmetro externo traseiro (mm)
            
            %% Parâmetros Ambientais
            vehicle.mu = 1.0;       % Coeficiente de aderência (asfalto seco)
            vehicle.g = 9.81;       % Aceleração da gravidade (m/s²)
            
            %% Operacional
            vehicle.v = 70 / 3.6;   % Velocidade longitudinal nominal (m/s)
            
            %% Métricas de Estabilidade
            vehicle.SSF = (vehicle.lw * 2) / (2 * vehicle.h); % Static Stability Factor
            
            %% Metadata
            vehicle.name = 'Chevrolet Blazer 2001';
            vehicle.reference = 'Código Base Funcional Anterior';
            vehicle.dof = 14; 
            
        otherwise
            error('VehicleParams:InvalidType', ...
                'Tipo de veículo inválido: \"%s\". Use ''truck''.', type);
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

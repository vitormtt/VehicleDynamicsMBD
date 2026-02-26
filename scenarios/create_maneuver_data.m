function steer_ts = create_maneuver_data(maneuver_type, sim_time, v_kmh)
%CREATE_MANEUVER_DATA Gera sinal de esterçamento em formato MBD (timeseries)
%
% Sintaxe:
%   steer_ts = create_maneuver_data(maneuver_type, sim_time, v_kmh)
%
% Entradas:
%   maneuver_type - 'DLC', 'Fishhook', 'J-Turn', 'StepSteer', 'Gaspar'
%   sim_time      - Tempo total de simulacao (s)
%   v_kmh         - Velocidade longitudinal (km/h) para calculos dependentes
%
% Saida:
%   steer_ts - timeseries object pronto para bloco 'From Workspace' no Simulink
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 26/02/2026

    dt = 0.01;
    t = (0:dt:sim_time)';
    steer = zeros(size(t));
    
    % Fator de esterçamento baseado no ISO e papers de referencia
    % Valores em radianos no pneu (ajuste steering ratio no Simulink se for no volante)
    
    switch maneuver_type
        case 'DLC'
            % Double Lane Change (ISO 3888)
            t_start = 1.0;
            for i = 1:length(t)
                if t(i) >= t_start && t(i) < t_start + 1.0
                    steer(i) = sin(2*pi*0.5*(t(i)-t_start)) * deg2rad(60);
                elseif t(i) >= t_start + 1.0 && t(i) < t_start + 2.0
                    steer(i) = -sin(2*pi*0.5*(t(i)-(t_start+1.0))) * deg2rad(60);
                end
            end
            
        case 'Gaspar'
            % Single Lane Change (Manobra padrão de validação Gaspar 2004)
            % Onda senoidal simples de período T = 3s (ou 2s)
            t_start = 1.0;
            period = 3.0;
            A = 0.05; % radianos nos pneus (~2.86 deg) comum na literatura de controle de HVs
            for i = 1:length(t)
                if t(i) >= t_start && t(i) < t_start + period
                    steer(i) = A * sin(2*pi*(1/period)*(t(i)-t_start));
                end
            end

        case 'Fishhook'
            % NHTSA Fishhook (FMVSS 126)
            A = deg2rad(294);
            for i = 1:length(t)
                if t(i) >= 1.0 && t(i) < 1.25
                    steer(i) = (t(i)-1.0)/0.25 * A;
                elseif t(i) >= 1.25 && t(i) < 1.5
                    steer(i) = A;
                elseif t(i) >= 1.5 && t(i) < 1.75
                    steer(i) = A - (t(i)-1.5)/0.25 * (2*A);
                elseif t(i) >= 1.75
                    steer(i) = -A;
                end
            end
            
        case 'J-Turn'
            % J-Turn step input
            A = deg2rad(100);
            for i = 1:length(t)
                if t(i) >= 1.0 && t(i) < 1.5
                    steer(i) = (t(i)-1.0)/0.5 * A;
                elseif t(i) >= 1.5
                    steer(i) = A;
                end
            end
            
        case 'StepSteer'
            % Degrau de esterçamento simples (ISO 7401)
            A = deg2rad(45);
            for i = 1:length(t)
                if t(i) >= 1.0
                    steer(i) = A;
                end
            end
            
        otherwise
            error('Manobra desconhecida: %s', maneuver_type);
    end
    
    % Criar objeto timeseries para injetar no Simulink de forma MBD-compliant
    steer_ts = timeseries(steer, t, 'Name', 'Steering_Angle_rad');
end

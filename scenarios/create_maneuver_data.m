function steer_ts = create_maneuver_data(maneuver_type, sim_time, v_kmh)
%CREATE_MANEUVER_DATA Gera sinal de esterçamento em formato MBD (timeseries)
    dt = 0.01;
    t = (0:dt:sim_time)';
    steer = zeros(size(t));
    
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
            t1 = 1;
            t2 = 2;
            t3 = 3.5;
            tf = 5;
            
            % Gera os três pulsos da manobra em graus
            steer = (t >= t1 & t < t2) .* (2 * sin(pi * (t - t1) / (t2 - t1))) + ...
                    (t >= t2 & t < t3) .* (-4 * sin(pi * (t - t2) / (t3 - t2))) + ...
                    (t >= t3 & t < tf) .* (2 * sin(pi * (t - t3) / (tf - t3)));
                    
            % PADRÃO MBD: O Workspace DEVE fornecer Radianos!
            % A matriz de pneu (Cf, Cr) é dada em N/rad.
            steer = deg2rad(steer);

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
            A = deg2rad(100);
            for i = 1:length(t)
                if t(i) >= 1.0 && t(i) < 1.5
                    steer(i) = (t(i)-1.0)/0.5 * A;
                elseif t(i) >= 1.5
                    steer(i) = A;
                end
            end
            
        case 'StepSteer'
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

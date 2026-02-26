% Function: create_steering_signal
% Description: Create a Double Lane Change maneuver compliant with Gaspar/Vu literature
function steering_signal = create_steering_signal(maneuver_type, vehicle)
    % Parâmetros do sinal e da simulação
    t_end = 10;
    dt = 0.01;
    t = (0:dt:t_end)';
    
    switch lower(maneuver_type)
        case 'gaspar'
            % Sinal do artigo Gaspar et al. (Double Lane Change modificado)
            delta = zeros(size(t));
            
            % Tempos (em s)
            t1 = 1;
            t2 = 2;
            t3 = 3.5;
            tf = 5;
            
            % Criação do sinal usando lógica booleana
            % Amplitude base do Gaspar = 2 radianos * FATOR_ESCALA para conversão em delta
            % No artigo os inputs variam de +/- 4 graus (aprox 0.07 rad) no volante
            % Como a entrada era um seno gigante no seu código antigo, era em Graus (deg), 
            % então precisa multiplicar por (pi/180) para entrar no cornering stiffness do pneu
            
            delta = (t >= t1 & t < t2) .* (2 * sin(pi * (t - t1) / (t2 - t1))) + ...
                    (t >= t2 & t < t3) .* (-4 * sin(pi * (t - t2) / (t3 - t2))) + ...
                    (t >= t3 & t < tf) .* (2 * sin(pi * (t - t3) / (tf - t3)));
                    
            % Convertendo de graus para radianos (assumindo que o código base era em deg)
            delta = delta * (pi / 180);
            
        case 'step'
            delta = [zeros(100,1); 2*(pi/180)*ones(length(t)-100,1)]; % Step de 2 deg
            
        otherwise
            error('Maneuver unknown');
    end
    
    % Formato timeseries para Simulink
    steering_signal = timeseries(delta, t);
end

function steering_signal = generate_dlc_maneuver(varargin)
%GENERATE_DLC_MANEUVER Gera perfil de esterçamento Double Lane Change
%
% Sintaxe:
%   steering_signal = generate_dlc_maneuver()
%   steering_signal = generate_dlc_maneuver(t1, t2, t3, tf, t_total)
%
% Entradas (opcionais):
%   t1      - Tempo inicial da 1ª curva (default: 1.0 s)
%   t2      - Tempo final da 1ª curva (default: 2.0 s)
%   t3      - Tempo final da 2ª curva (default: 3.5 s)
%   tf      - Tempo final da 3ª curva (default: 5.0 s)
%   t_total - Tempo total de simulação (default: 10.0 s)
%
% Saídas:
%   steering_signal - Array [T×2] com [tempo, ângulo_rad]
%
% Referência:
%   ISO 3888-1:2018 - Passenger cars - Test track for a severe lane-change
%
% Exemplo:
%   sig = generate_dlc_maneuver(1, 2, 3.5, 5, 10);
%   plot(sig(:,1), rad2deg(sig(:,2)));
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    % Parser de argumentos
    p = inputParser;
    addOptional(p, 't1', 1.0, @(x) x>0);
    addOptional(p, 't2', 2.0, @(x) x>0);
    addOptional(p, 't3', 3.5, @(x) x>0);
    addOptional(p, 'tf', 5.0, @(x) x>0);
    addOptional(p, 't_total', 10.0, @(x) x>0);
    parse(p, varargin{:});
    
    t1 = p.Results.t1;
    t2 = p.Results.t2;
    t3 = p.Results.t3;
    tf = p.Results.tf;
    t_total = p.Results.t_total;
    
    % Validação de sequência temporal
    assert(t1 < t2 && t2 < t3 && t3 < tf && tf < t_total, ...
        'Tempos devem seguir: t1 < t2 < t3 < tf < t_total');
    
    % Vetor de tempo
    t = linspace(0, t_total, 1000)';
    
    % Perfil sinusoidal de esterçamento (deg → rad)
    delta_deg = (t >= t1 & t < t2) .* (2 * sin(pi * (t - t1) / (t2 - t1))) + ...
                (t >= t2 & t < t3) .* (-4 * sin(pi * (t - t2) / (t3 - t2))) + ...
                (t >= t3 & t < tf) .* (2 * sin(pi * (t - t3) / (tf - t3)));
    
    delta_rad = deg2rad(delta_deg);
    
    % Output format: [time, steering_angle]
    steering_signal = [t, delta_rad];
end

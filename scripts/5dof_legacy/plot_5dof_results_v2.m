function plot_5dof_results_v2(simOut, metadata)
%PLOT_5DOF_RESULTS_V2 Plota resultados da simulação (MBD Standard)
%
% Autor: Vitor Yukio

try
    logs = simOut.logsout;
    
    t = logs.get('phi').Values.Time;
    phi = logs.get('phi').Values.Data * (180/pi);
    ay = logs.get('ay').Values.Data;
    nlt_f = logs.get('NLT_f').Values.Data;
    nlt_r = logs.get('NLT_r').Values.Data;
    
    try
        Tf = logs.get('T_f').Values.Data;
        Tr = logs.get('T_r').Values.Data;
    catch
        Tf = zeros(size(t));
        Tr = zeros(size(t));
    end
catch
    warning('Nao foi possivel extrair sinais do logsout para plotagem.');
    return;
end

figure('Name', sprintf('Results: %s - %s', metadata.controller, metadata.maneuver), ...
       'Position', [100, 100, 1000, 800], 'Color', 'w');

% 1. Roll Angle
subplot(2,2,1);
plot(t, phi, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
grid on; box on;
title('Roll Angle (\phi)');
ylabel('[deg]');

% 2. Lateral Acceleration
subplot(2,2,2);
plot(t, ay, 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980]);
grid on; box on;
title('Lateral Acceleration (a_y)');
ylabel('[m/s^2]');

% 3. NLT
subplot(2,2,3);
plot(t, nlt_f, 'LineWidth', 1.5, 'DisplayName', 'Front');
hold on;
plot(t, nlt_r, 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Rear');
yline(1, 'r-', 'Liftoff Limit');
yline(-1, 'r-');
grid on; box on;
title('Normalized Load Transfer');
ylabel('[-]');
legend('Location', 'best');
ylim([-1.2 1.2]);

% 4. Control Torques
subplot(2,2,4);
plot(t, Tf, 'LineWidth', 1.5, 'DisplayName', 'T_{front}');
hold on;
plot(t, Tr, 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'T_{rear}');
grid on; box on;
title('Active Anti-Roll Bar Torque');
xlabel('Time [s]');
ylabel('[Nm]');
legend('Location', 'best');

sgtitle(sprintf('Simulation: %s | %s | %.1f km/h', ...
        metadata.controller, metadata.maneuver, metadata.velocity_kmh), ...
        'FontWeight', 'bold');

end
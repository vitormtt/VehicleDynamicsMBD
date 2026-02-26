% validation/analyze_results.m
function analyze_results(simOut_passive, simOut_pid, simOut_lqr)
    % Extrai states de logsout (Dataset)
    passive_states = simOut_passive.logsout.get('vehicle_states').Values;
    pid_states = simOut_pid.logsout.get('vehicle_states').Values;
    lqr_states = simOut_lqr.logsout.get('vehicle_states').Values;
    
    % Plot comparação
    figure('Position', [100 100 1200 800]);
    
    states_names = {'\beta (°)', 'r (°/s)', '\phi (°)'};
    for i = 1:3
        subplot(3,1,i);
        plot(passive_states.Time, rad2deg(passive_states.Data(:,i)), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(pid_states.Time, rad2deg(pid_states.Data(:,i)), 'r--', 'LineWidth', 1.5);
        plot(lqr_states.Time, rad2deg(lqr_states.Data(:,i)), 'g:', 'LineWidth', 1.5);
        ylabel(states_names{i}); grid on;
        if i == 1, legend('Passive', 'PID', 'LQR', 'Location', 'best'); end
    end
    xlabel('Time (s)');
    sgtitle('Comparação Controladores - Fishhook 64.4 km/h');
    
    % Métricas ISO 19364
    rmse_pid_beta = rms(passive_states.Data(:,1) - pid_states.Data(:,1));
    rmse_lqr_beta = rms(passive_states.Data(:,1) - lqr_states.Data(:,1));
    
    fprintf('\n📊 VALIDAÇÃO ISO 19364\n');
    fprintf('RMSE β (Passive vs PID): %.4f rad\n', rmse_pid_beta);
    fprintf('RMSE β (Passive vs LQR): %.4f rad\n', rmse_lqr_beta);
    
    % Simulink Data Inspector (comparação visual)
    Simulink.sdi.view;
    Simulink.sdi.clear;
    run1 = Simulink.sdi.Run.create('Passive');
    run1.add(passive_states);
    run2 = Simulink.sdi.Run.create('PID');
    run2.add(pid_states);
    run3 = Simulink.sdi.Run.create('LQR');
    run3.add(lqr_states);
end

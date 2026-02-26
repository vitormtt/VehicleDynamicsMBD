function results = run_comparison(model, maneuvers, speeds)
    % Exemplo: run_comparison('5dof_model', {'DLC','Fishhook'}, [60,70,80])
    controllers = {'Passive', 'PID', 'LQR'};
    results = struct();
    
    parfor idx = 1:numel(controllers)*numel(maneuvers)*numel(speeds)
        [c,m,v] = ind2sub(...); % Mapear índice linear
        out = sim(model, 'ARB_Mode', c, 'Scenario', m, 'Speed', v);
        results(idx) = extract_metrics(out);
    end
    
    % Gerar relatório comparativo
    generate_comparison_table(results);
end

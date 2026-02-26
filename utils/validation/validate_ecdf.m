function [ks_stat, p_value] = validate_ecdf(sim_data, exp_data)
    [~,p_value,ks_stat] = kstest2(sim_data, exp_data, 'Alpha', 0.05);
    % Critério: p_value > 0.05 (não rejeitar H0)
end

% objective_mpc_tuning.m
% Descrição: Função-objetivo para otimização dos pesos do controlador MPC.

function J = objective_mpc_tuning(mpc_weights, sys, params, t, u_maneuver)
    % mpc_weights: vetor com os pesos a serem otimizados, e.g., [q_phi, r_T_arb]

    % 1. Extrair pesos
    q_phi = mpc_weights(1);
    r_T_arb = mpc_weights(2);

    % 2. Construir o MPC
    % Aqui você usaria o Model Predictive Control Toolbox para definir o
    % objeto MPC com os pesos atuais.
    % Q = diag([... q_phi ...]); R = diag([... r_T_arb ...]);
    % mpc_obj = mpc(sys, dt, p, m, Q, R, ...);
    
    % 3. Simular o sistema em malha fechada
    % [y_cl, ~, u_cl] = lsim(mpc_obj, ref_signal, t);
    % Por simplicidade, faremos uma simulação em malha aberta com uma lei
    % de controle placeholder para demonstrar o cálculo do custo.
    [y_sim, ~, x_sim] = lsim(sys, u_maneuver, t); % Simulação de referência
    
    % 4. Calcular o Custo J
    phi = y_sim(:,3);
    T_arb_f = u_maneuver(:,2); % Em uma simulação real, viria de 'u_cl'

    cost_performance = rms(phi);    % Custo de performance (minimizar rolagem)
    cost_effort = rms(T_arb_f); % Custo de esforço de controle
    
    % Custo total ponderado
    J = 10 * cost_performance + 1 * cost_effort;
end
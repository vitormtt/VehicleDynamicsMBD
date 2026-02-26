function mpcobj = create_mpc_controller(sys, Ts)
%CREATE_MPC_CONTROLLER Desenha e configura o controlador MPC para AARB
%
%   Esta função implementa o controlador Model Predictive Control (MPC) 
%   focado na mitigação de capotamento e estabilidade direcional usando 
%   Active Anti-Roll Bars (AARBs) independentes na dianteira e traseira.
%
%   INPUTS:
%       sys - Objeto State-Space (ss) da planta do veículo.
%       Ts  - Tempo de amostragem do controlador (ex: 0.01 s).
%
%   OUTPUT:
%       mpcobj - Objeto MPC da toolbox configurado para uso no Simulink.
%
%   MVs (Manipulated Variables - 2 MVs):
%       u(1): Torque ARB Dianteiro (T_arb_f)
%       u(2): Torque ARB Traseiro (T_arb_r)
%
%   OVs (Measured Outputs - Mínimo 4 recomendadas pela literatura):
%       y(1): Roll Angle (phi)
%       y(2): Roll Rate (p)
%       y(3): Yaw Rate (r)
%       y(4): Sideslip Angle (beta)
%
%   REFERÊNCIAS:
%       [1] Khalil, M. et al. (2019). T_max = 15kNm, dT_max = 50kNm/s.
%       [2] Gaspar, P. et al. (2004). Rollover prevention via LPV/MPC.
%
%   Autor: Vitor Yukio (UnB/PIBIC)

    %% 1. Verificação do Modelo Planta
    if nargin < 2
        Ts = 0.01; % Tempo de amostragem padrão (100 Hz)
    end
    
    % Se 'sys' não for fornecido, cria um SS dummy (Apenas para MBD linting)
    % Na prática, essa função deve receber sys = ss(A, B, C, D) do seu D.D.
    if nargin < 1
        warning('Modelo State-Space nao fornecido. Usando dummy para instanciacao.');
        sys = ss(eye(6), ones(6,2), eye(4,6), zeros(4,2));
    end

    %% 2. Inicialização do Objeto MPC
    % Definir MVs (entradas de controle) e MDs (disturbios, ex: steer) se houver.
    % Aqui assumimos planta padrao sys: u = [T_f; T_r], y = [phi; p; r; beta]
    mpcobj = mpc(sys, Ts);

    %% 3. Horizontes de Predição e Controle
    % Np: Suficiente para cobrir a dinamica de rolagem (frequencia natural ~1.5 Hz)
    mpcobj.PredictionHorizon = 30; % 30 * 0.01s = 0.3s
    mpcobj.ControlHorizon = 5;     % Conservador para reduzir esforco computacional

    %% 4. Limites Físicos (Constraints)
    % Baseado em literatura para Heavy Vehicles / SUVs: T_max = 15 kNm
    max_torque = 15000;      % [Nm]
    max_torque_rate = 50000; % [Nm/s] -> equivale a 500 Nm/passo (se Ts=0.01)

    for i = 1:2
        mpcobj.MV(i).Min = -max_torque;
        mpcobj.MV(i).Max = max_torque;
        mpcobj.MV(i).RateMin = -max_torque_rate * Ts;
        mpcobj.MV(i).RateMax = max_torque_rate * Ts;
    end

    %% 5. Pesos da Função Custo (Tuning)
    % Ajuste fino (Tuning Matrix Q e R)
    % Q: Penaliza as saidas. Prioridade absoluta: phi.
    % R: Penaliza a atuacao (Rate).
    
    % Pesos OVs: [phi, p, r, beta]
    % Escalonamento natural: radianos tem valores absolutos baixos (0.02 rad = 1 deg).
    % Pesos devem ser altos para phi.
    mpcobj.Weights.OV = [1000, 10, 50, 100];
    
    % Pesos MVs Rate (dT): Penalidade para uso agressivo dos motores do AARB
    mpcobj.Weights.ManipulatedVariablesRate = [0.1, 0.1];
    
    % Pesos MVs absolutos: Proximo a zero, prefere-se focar na variacao (Rate)
    mpcobj.Weights.ManipulatedVariables = [0, 0];

    fprintf('   ✅ Controlador MPC instanciado com sucesso (Np=%d, Nc=%d, Ts=%.2fs)\\n', ...
        mpcobj.PredictionHorizon, mpcobj.ControlHorizon, Ts);
end

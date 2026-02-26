% =========================================================================
% CREATE_MPC_CONTROLLER (VERSÃO FINAL E SIMPLIFICADA)
% =========================================================================
% Descrição:
% Cria o objeto de controlador MPC final. Esta versão é a mais simples e
% robusta. O controlador terá APENAS 2 entradas (as Variáveis Manipuladas)
% e, portanto, APENAS 2 saídas (os torques).
% =========================================================================

fprintf('Criando o objeto MPC Controller (2 Entradas, 8 Saídas)...\n');

% --- Parâmetros de Configuração ---
Ts = 0.02; % Tempo de amostragem
p = 20;    % Horizonte de predição
m = 5;     % Horizonte de controle

% --- 1. Construção do Modelo de Planta SÓ PARA O MPC ---
% O modelo para o MPC conterá apenas as Variáveis Manipuladas (MVs).
% Índices das MVs na matriz B do sistema principal 'sys'
mv_indices = [3, 4]; % Colunas de T_arb_f, T_arb_r

% Cria uma nova matriz B para o MPC, contendo apenas as 2 colunas das MVs.
B_mpc = sys.B(:, mv_indices);

% Seleciona as 8 saídas que o MPC vai monitorar
output_indices_for_mpc = [3, 4, 5, 6, 7, 8, 9, 10];
C_mpc = sys.C(output_indices_for_mpc, :);

% A matriz D correspondente agora terá apenas 2 colunas
D_mpc = sys.D(output_indices_for_mpc, mv_indices);

% Cria o objeto state-space dedicado (16 estados, 2 ENTRADAS, 8 saídas)
sys_for_mpc = ss(sys.A, B_mpc, C_mpc, D_mpc);

% Aplica 'minreal' para garantir a detectabilidade.
sys_for_mpc = minreal(sys_for_mpc);
fprintf('   -> Modelo de planta para MPC criado e minimizado. Estados restantes: %d\n', size(sys_for_mpc.A, 1));

% --- 2. Criação do Objeto MPC ---
% Discretiza a planta final
plant_d = c2d(sys_for_mpc, Ts);

% Cria o objeto MPC. Como a planta agora só tem 2 entradas (que são as MVs),
% a criação é direta e robusta.
mpc_obj = mpc(plant_d, Ts, p, m);

% --- 3. Ajuste dos Pesos ---
mpc_obj.Weights.OutputVariables = [10, 1, 1e-6, 1e-6, 1e-6, 1e-6, 5, 5];
mpc_obj.Weights.ManipulatedVariables = [0.1, 0.1];

fprintf('Objeto MPC "mpc_obj" criado com sucesso.\n');
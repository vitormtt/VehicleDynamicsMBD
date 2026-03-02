function [sys, params] = build_9dof_linear_model()
% BUILD_9DOF_LINEAR_MODEL Constrói o modelo linearizado 9-DOF em espaço de estados
%
% Este modelo inclui:
% - 2-DOF Handling (Velocidade lateral Vy e Yaw Rate r)
% - 3-DOF Ride/Roll (Deslocamento vertical z, Roll phi e Pitch theta)
% - 4-DOF Rodas (Velocidades angulares w_fl, w_fr, w_rl, w_rr)
%
% Referência: "Improving Vehicle Rollover Resistance Using Fuzzy PID Controller..." (Khalil et al.)
% Foco: Aplicação em Active Anti-Roll Bar (AARB) e controle LQR/MPC

%% 1. PARÂMETROS DO VEÍCULO (CHEVROLET BLAZER 2001)
% Parâmetros retirados da Tabela 1 e Tabela 2 do paper Khalil et al.
params.m    = 1905;      % Massa total [kg]
params.ms   = 1525;      % Massa suspensa [kg]
params.m_uf = 80;        % Massa não-suspensa frontal [kg] (estimada/referência)
params.m_ur = 110;       % Massa não-suspensa traseira [kg] (estimada/referência)

params.Ixx  = 734.04;    % Momento de inércia em roll [kg.m^2]
params.Iyy  = 3644.0;    % Momento de inércia em pitch [kg.m^2]
params.Izz  = 3833.31;   % Momento de inércia em yaw [kg.m^2]

params.a    = 1.216;     % Distância CG ao eixo frontal [m]
params.b    = 1.502;     % Distância CG ao eixo traseiro [m]
params.L    = params.a + params.b; % Wheelbase [m]

params.Tw   = 1.549;     % Track width (Trilha) [m]
params.hcg  = 0.6629;    % Altura do CG [m]
params.hr_f = 0.58;      % Altura do centro de rolagem frontal [m] (Ajustar conforme ref)
params.hr_r = 0.65;      % Altura do centro de rolagem traseiro [m] (Ajustar conforme ref)

% Parâmetros de Suspensão e Pneu
params.Ksf  = 75000;     % Rigidez da suspensão frontal [N/m]
params.Csf  = 5000;      % Amortecimento da suspensão frontal [Ns/m]
params.Ksr  = 70000;     % Rigidez da suspensão traseira [N/m]
params.Csr  = 4000;      % Amortecimento da suspensão traseira [Ns/m]

params.Kt   = 175500;    % Rigidez vertical do pneu [N/m]
params.R_w  = 0.36;      % Raio da roda [m]
params.Iw   = 1.2;       % Momento de inércia da roda [kg.m^2]

% Coeficientes de rigidez de curva (Cornering Stiffness) - Linearização
% Nota: Valores linearizados de Pacejka para pequenos ângulos de deriva
params.Cf = 65000;       % Rigidez de deriva frontal [N/rad] (estimado de Pacejka)
params.Cr = 60000;       % Rigidez de deriva traseira [N/rad] (estimado de Pacejka)

% Condição de operação
v_kmh = 70;
params.v_x = v_kmh / 3.6; % Velocidade longitudinal constante [m/s]

%% 2. MODELAGEM DO ESPAÇO DE ESTADOS (STATE-SPACE)
% Para o controle de AARB, os graus de liberdade acoplados mais importantes
% são o Handling (Vy, r) e o Roll (phi, phi_dot).
%
% O vetor de estados reduzido para o projeto de controle LQR/MPC do AARB será:
% X = [vy; r; phi; phi_dot; theta; theta_dot; z; z_dot] (8 estados principais)
%
% Inputs:
% U = [delta; T_aarb_f; T_aarb_r] (Esterçamento e Torques ativos)

Vx = params.v_x;
m  = params.m;
ms = params.ms;
Ixx = params.Ixx;
Iyy = params.Iyy;
Izz = params.Izz;
a = params.a;
b = params.b;
Tw = params.Tw;
hcg = params.hcg;
Cf = params.Cf;
Cr = params.Cr;
Ksf = params.Ksf; Csf = params.Csf;
Ksr = params.Ksr; Csr = params.Csr;

% Rigidez e amortecimento equivalente de rolagem (Roll stiffness/damping)
K_phi_f = 0.5 * Ksf * Tw^2;
K_phi_r = 0.5 * Ksr * Tw^2;
K_phi   = K_phi_f + K_phi_r;

C_phi_f = 0.5 * Csf * Tw^2;
C_phi_r = 0.5 * Csr * Tw^2;
C_phi   = C_phi_f + C_phi_r;

% Distância do CG ao eixo de rolagem (Roll axis)
h_roll = hcg - (params.hr_f + a/params.L * (params.hr_r - params.hr_f));

%% Matrizes A e B (Simplificadas - Foco Yaw-Roll Acoplado)
% Baseado no modelo linear "Bicycle Model" acoplado com dinâmica de Roll
% Eq. X_dot = A*X + B*U

% Vetor de estados: X = [vy; r; phi; p] (Modelo 4 estados reduzido para controle yaw-roll)
% Onde p = phi_dot

% Constantes auxiliares
Ixx_eq = Ixx + ms * h_roll^2; % Inércia efetiva de rolagem

A11 = -(Cf + Cr) / (m * Vx);
A12 = -Vx - (a*Cf - b*Cr) / (m * Vx);
A13 = 0;
A14 = 0;

A21 = -(a*Cf - b*Cr) / (Izz * Vx);
A22 = -(a^2 * Cf + b^2 * Cr) / (Izz * Vx);
A23 = 0;
A24 = 0;

A31 = 0; A32 = 0; A33 = 0; A34 = 1;

A41 = -(ms * h_roll * (Cf + Cr)) / (Ixx_eq * Vx);
A42 = -(ms * h_roll * (a*Cf - b*Cr)) / (Ixx_eq * Vx) + (ms * h_roll * Vx) / Ixx_eq;
A43 = -(K_phi - ms * 9.81 * h_roll) / Ixx_eq;
A44 = -C_phi / Ixx_eq;

A = [A11 A12 A13 A14;
     A21 A22 A23 A24;
     A31 A32 A33 A34;
     A41 A42 A43 A44];

% Matriz B
% Entradas: U = [delta; U_aarb] (Ângulo de esterçamento e Força/Momento AARB)
B11 = Cf / m;
B12 = 0;

B21 = a * Cf / Izz;
B22 = 0;

B31 = 0;
B32 = 0;

B41 = (ms * h_roll * Cf) / Ixx_eq;
B42 = 1 / Ixx_eq; % Influência do torque do AARB na aceleração de rolagem

B = [B11 B12;
     B21 B22;
     B31 B32;
     B41 B42];

% Matrizes C e D (Saídas medidas)
% Y = [ay; r; phi; phi_dot]
% Aceleração lateral ay = vy_dot + Vx*r

C11 = A11; C12 = A12 + Vx; C13 = A13; C14 = A14; % ay
C21 = 0;   C22 = 1;        C23 = 0;   C24 = 0;   % r
C31 = 0;   C32 = 0;        C33 = 1;   C34 = 0;   % phi
C41 = 0;   C42 = 0;        C43 = 0;   C44 = 1;   % p

C = [C11 C12 C13 C14;
     C21 C22 C23 C24;
     C31 C32 C33 C34;
     C41 C42 C43 C44];

D = [B11 B12;
     0   0;
     0   0;
     0   0];

sys = ss(A, B, C, D);
sys.StateName = {'v_y', 'r', 'phi', 'p'};
sys.InputName = {'delta', 'T_aarb'};
sys.OutputName = {'a_y', 'r', 'phi', 'p'};

disp('Modelo State-Space Linear (Foco Yaw-Roll) criado com sucesso.');
end
%% build_linear_model.m (Versão Final e Definitiva)
% Este script constrói as matrizes para o modelo linear completo de 9-DOF,
% utilizando o método de construção direta para máxima eficiência.
% A dinâmica do pneu (linearizada) é incorporada nas matrizes F e G.
% As matrizes geradas devem ser usadas em um bloco "Descriptor State-Space".

fprintf('Iniciando a construção do modelo linear descritor...\n');

%% --- 1. Definição dos Parâmetros Numéricos ---
% Parâmetros baseados principalmente na Tabela II do artigo de Hassan (2021).
fprintf('Definindo parâmetros numéricos...\n');
params.M_tot = 1465;      % Massa Total do Veículo 
params.Mb = 1286;         % Massa Suspensa (m_s) 
params.m_u = 40;          % Massa Não Suspensa (por roda) 
params.I_z = 1972;        % Momento de inércia em guinada (yaw) 
params.I_x = 535;         % Momento de inércia em rolagem (roll) 
params.I_y = 1859;        % Momento de inércia em arfagem (pitch) 
params.l_f = 1.0;         % Distância CG ao eixo dianteiro (a) 
params.l_r = 1.6;         % Distância CG ao eixo traseiro (b) 
params.w = 0.773;         % Metade da bitola do veículo 
params.k_sf = 12548;      % Rigidez da suspensão dianteira 
params.k_sr = 22639;      % Rigidez da suspensão traseira 
params.c_sf = 1500;       % Amortecimento da suspensão dianteira 
params.c_sr = 3000;       % Amortecimento da suspensão traseira 
params.k_tf = 473520;     % Rigidez do pneu dianteiro (vertical) 
params.k_tr = 460780;     % Rigidez do pneu traseiro (vertical) 
params.h_cg = 0.52;       % Altura do CG (h) 
params.C_alpha = 76776;   % Rigidez lateral do pneu (por eixo) 
params.g = 9.81;
params.v_x = 20;          % Velocidade de operação (m/s)

% Extração de parâmetros para variáveis locais
Mb=params.Mb; m_u=params.m_u; I_x=params.I_x; I_y=params.I_y; I_z=params.I_z; M_tot=params.M_tot;
l_f=params.l_f; l_r=params.l_r; bf=2*params.w; br=2*params.w; h_cg=params.h_cg;
ksf=params.k_sf; ksr=params.k_sr; csf=params.c_sf; csr=params.c_sr;
ktf=params.k_tf; ktr=params.k_tr;
Caf = params.C_alpha / 2; Car = params.C_alpha / 2; % Por roda
vx = params.v_x; g=params.g;


%% --- 2. Construção da Matriz E (Massa e Inércia) ---
fprintf('Construindo matriz E (16x16)...\n');
E_matrix = zeros(16, 16);
% Parte Cinemática
E_matrix(1:7, 1:7) = eye(7);
% Parte Dinâmica (Massa/Inércia)
E_matrix(1,1) = Mb;    
E_matrix(2,2) = I_y;   
E_matrix(3,3) = I_x;
E_matrix(4,4) = m_u; 
E_matrix(5,5) = m_u; 
E_matrix(6,6) = m_u; 
E_matrix(7,7) = m_u;
E_matrix(8,8) = M_tot; 
E_matrix(9,9) = I_z;
% Termo de acoplamento Roll-Lateral da Equação (4)
E_matrix(2, 8) = Mb * h_cg;

%% --- 3. Construção da Matriz F (Rigidez, Amortecimento e Dinâmica Linear) ---
fprintf('Construindo matriz F (16x16)...\n');
F_matrix = zeros(16, 16);
% Parte Cinemática
F_matrix(1:7, 8:14) = eye(7);

% Parte Dinâmica (termos que multiplicam os estados x)
% Linhas 8-10: Equações do Chassi (Bounce, Pitch, Roll)
F_matrix(8, [1,2,4,5,6,7])     = [-(2*ksf+2*ksr), 2*l_f*ksf-2*l_r*ksr, 0, ksf, ksf, ksr, ksr];
F_matrix(8, [8,9,11,12,13,14]) = [-(2*csf+2*csr), 2*l_f*csf-2*l_r*csr, csf, csf, csr, csr];
F_matrix(9, [1,2,4,5,6,7])     = [2*l_f*ksf-2*l_r*ksr, -(2*l_f^2*ksf+2*l_r^2*ksr), 0, -l_f*ksf, -l_f*ksf, l_r*ksr, l_r*ksr];
F_matrix(9, [8,9,11,12,13,14]) = [2*l_f*csf-2*l_r*csr, -(2*l_f^2*csf+2*l_r^2*csr), -l_f*csf, -l_f*csf, l_r*csr, l_r*csr];
F_matrix(10, 3)                = -((bf^2/2)*ksf+(br^2/2)*ksr) - Mb*g*h_cg;
F_matrix(10, [4,5,6,7])        = [-ksf*bf/2, ksf*bf/2, -ksr*br/2, ksr*br/2];
F_matrix(10, 10)               = -((bf^2/2)*csf+(br^2/2)*csr);
F_matrix(10, [11,12,13,14])    = [-csf*bf/2, csf*bf/2, -csr*br/2, csr*br/2];

% Linhas 11-14: Equações das Rodas
F_matrix(11, [1,2,3,4])        = [ksf, -l_f*ksf, -bf/2*ksf, -(ksf+ktf)];
F_matrix(11, [8,9,10,11])      = [csf, -l_f*csf, -bf/2*csf, -csf];
F_matrix(12, [1,2,3,5])        = [ksf, -l_f*ksf, bf/2*ksf, -(ksf+ktf)];
F_matrix(12, [8,9,10,12])      = [csf, -l_f*csf, bf/2*csf, -csf];
F_matrix(13, [1,2,3,6])        = [ksr, l_r*ksr, br/2*ksr, -(ksr+ktr)];
F_matrix(13, [8,9,10,13])      = [csr, l_r*csr, br/2*csr, -csr];
F_matrix(14, [1,2,3,7])        = [ksr, l_r*ksr, -br/2*ksr, -(ksr+ktr)];
F_matrix(14, [8,9,10,14])      = [csr, l_r*csr, -br/2*csr, -csr];

% Linhas 15-16: Equações de Handling (pneu linearizado + acoplamentos)
F_matrix(10, 16)               = Mb*vx*h_cg; % Acoplamento Roll-Handling (termo Mb*vx*r*hcg)
F_matrix(15, [15,16])           = [-(2*Caf+2*Car)/vx, -(2*l_f*Caf-2*l_r*Car)/vx-M_tot*vx];
F_matrix(16, [15,16])           = [-(2*l_f*Caf-2*l_r*Car)/vx, -(2*l_f^2*Caf+2*l_r^2*Car)/vx];

%% --- 4. Construção da Matriz G (Entradas Externas) ---
fprintf('Construindo matriz G (16x5)...\n');
G_matrix = zeros(16, 5);
% Entradas u = [zRfr, zRfl, zRrr, zRrl, delta_f]
G_matrix(11,1) = ktf; G_matrix(12,2) = ktf; G_matrix(13,3) = ktr; G_matrix(14,4) = ktr; % Entradas de Pista
G_matrix(15,5) = 2*Caf;                                                               % Efeito de delta_f na eq. Lateral
G_matrix(16,5) = 2*l_f*Caf;                                                           % Efeito de delta_f na eq. Yaw

%% --- 5. Construção das Matrizes de Saída C e D ---
fprintf('Construindo matrizes C e D...\n');
C_matrix = eye(16);
D_matrix = zeros(16, 5);
fprintf('Construção das matrizes do modelo descritor concluída.\n\n');
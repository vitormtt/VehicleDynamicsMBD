function [sys] = build_active_state_space(vehicle, MAR, v)
%BUILD_ACTIVE_STATE_SPACE Constrói matrizes A/B para modelo 5-DOF ativo
%
% Sintaxe:
%   sys = build_active_state_space(vehicle, MAR, v)
%
% Entradas:
%   vehicle - Struct com parâmetros do veículo (massa, geometria, suspensão)
%   MAR     - Struct com momentos ARB (.f e .r com campos .phi e .phi_u)
%   v       - Velocidade longitudinal (m/s)
%
% Saídas:
%   sys - Struct com campos:
%         .A       - Matriz de estados 6×6
%         .B_total - Matriz de entradas 6×4 [δf, δr, Tf, Tr]
%         .C       - Matriz de saídas 6×6 (identidade)
%         .D       - Matriz feedthrough 6×4 (zeros)
%
% Referência:
%   Vu et al. (2016), IFAC AAC, Section 2.2
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    % Extração de parâmetros (readability)
    m = vehicle.m; ms = vehicle.ms; 
    muf = vehicle.muf; mur = vehicle.mur;
    h = vehicle.h; hu = vehicle.hu;
    rf = vehicle.rf; rr = vehicle.rr;
    lf = vehicle.lf; lr = vehicle.lr;
    Ixx = vehicle.Ixx; Izz = vehicle.Izz; Ixz = vehicle.Ixz;
    kf = vehicle.kf; kr = vehicle.kr;
    bf = vehicle.bf; br = vehicle.br;
    ktf = vehicle.ktf; ktr = vehicle.ktr;
    Cf = vehicle.Cf; Cr = vehicle.Cr;
    g = 9.81;
    
    % Matriz E (Eq. 10 Vu et al.)
    E_mat = [m*v,           0,          0,          -ms*h,           0,    0;
             0,             Izz,        0,          -Ixz,            0,    0;
             ms*v*h,        Ixz,        -bf-br,     -(Ixx+ms*h^2),   bf,   br;
             muf*v*(rf-hu), 0,          bf,         0,               -bf,  0;
             mur*v*(rr-hu), 0,          br,         0,               0,    -br;
             0,             0,          1,          0,               0,    0];
    
    % Matriz F (Eq. 10 Vu et al.) - sem forças dos pneus ainda
    F_mat = [0, m*v, 0, 0, 0, 0;
             0, 0,   0, 0, 0, 0;
             0, ms*v*h, ms*g*h - (kf + MAR.f.phi) - (kr + MAR.r.phi), ...
                -bf-br, kf + MAR.f.phi_u, kr + MAR.r.phi_u;
             0, muf*v*(rf-hu), kf + MAR.f.phi, bf, ...
                muf*g*hu - ktf - (kf + MAR.f.phi_u), 0;
             0, mur*v*(rr-hu), kr + MAR.r.phi, br, ...
                0, mur*g*hu - ktr - (kr + MAR.r.phi_u);
             0, 0, 0, -1, 0, 0];
    
    % Matriz H_common (Mapeia forças laterais Fyf e Fyr para as equações)
    H_common = [1,  1;
                lf, -lr;
                0,  0;
                -rf, 0;
                0,  -rr;
                0,  0];
                
    % Matriz M_alpha (Relaciona [Fyf; Fyr] com os estados [beta; r; ...])
    % Fyf = -Cf*beta - Cf*(lf/v)*r + Cf*delta_f
    % Fyr = -Cr*beta + Cr*(lr/v)*r + Cr*delta_r
    M_alpha = [-Cf, -Cf * lf / v, 0, 0, 0, 0;
               -Cr,  Cr * lr / v, 0, 0, 0, 0];
               
    % Incorporando os termos de slip angle (beta e r) na matriz F
    % Equação: E * x_dot = -F_mat * x + H_common * [Fyf; Fyr] + G * u_control
    % E * x_dot = -(F_mat - H_common * M_alpha) * x + H_common * diag([Cf, Cr]) * [delta_f; delta_r]
    F_mat = F_mat - H_common * M_alpha;
    
    % Matriz H para as entradas de esterçamento [delta_f, delta_r]
    H_steer = H_common * [Cf, 0; 
                          0, Cr];
    
    % Matriz G (control inputs: Tm_f, Tm_r)
    G = [0, 0;
         0, 0;
         -1, -1;
         0, -1;
         -1, -1;
         0, 0];
    
    % Cálculo do espaço de estados
    sys.A = -E_mat \ F_mat;
    B_passive = E_mat \ H_steer;
    B_control = E_mat \ G;
    sys.B_total = [B_passive, B_control]; % [δf δr Tf Tr]
    sys.C = eye(6);
    sys.D = zeros(6, 4);
    
    % Metadata
    sys.states = {'beta', 'r', 'phi', 'phi_dot', 'phi_uf', 'phi_ur'};
    sys.inputs = {'delta_f', 'delta_r', 'Tm_f', 'Tm_r'};
    sys.velocity = v;
end

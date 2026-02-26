function [sys] = build_active_state_space(vehicle, MAR, v)
%BUILD_ACTIVE_STATE_SPACE Constrói matrizes A/B para modelo 5-DOF ativo
%
% Refatorado para corresponder exatamente à formulação original.
% As forças dos pneus (Fyf, Fyr) são calculadas externamente no Simulink.
% Entradas do State-Space: [Fyf, Fyr, Tm_f, Tm_r]

    % Extração de parâmetros
    m = vehicle.m; ms = vehicle.ms; 
    muf = vehicle.muf; mur = vehicle.mur;
    h = vehicle.h; hu = vehicle.hu;
    rf = vehicle.rf; rr = vehicle.rr;
    lf = vehicle.lf; lr = vehicle.lr;
    Ixx = vehicle.Ixx; Izz = vehicle.Izz; Ixz = vehicle.Ixz;
    kf = vehicle.kf; kr = vehicle.kr;
    bf = vehicle.bf; br = vehicle.br;
    ktf = vehicle.ktf; ktr = vehicle.ktr;
    g = 9.81;
    
    % Momentos ARB Passiva
    MARf_phi = MAR.f.phi;
    MARr_phi = MAR.r.phi;
    MARf_phi_uf = MAR.f.phi_u;
    MARr_phi_ur = MAR.r.phi_u;
    
    % Definição da Matriz E
    E_mat = [ m*v,           0,   0,        -ms*h,             0,   0;
              0,             Izz, 0,        -Ixz,              0,   0;
              ms*v*h,        Ixz, -bf - br, -(Ixx + ms*(h^2)), bf,  br;
              muf*v*(rf-hu), 0,   bf,       0,                 -bf, 0;
              mur*v*(rr-hu), 0,   br,       0,                 0,   -br;
              0,             0,   1,        0,                 0,   0];
              
    % Definição da Matriz F
    F_mat = [ 0, m*v,           0,                                             0,        0,                                     0;
              0, 0,             0,                                             0,        0,                                     0;
              0, ms*v*h,        (ms*g*h) - (kf + MARf_phi) - (kr + MARr_phi), -bf - br, (kf + MARf_phi_uf),                    (kr + MARr_phi_ur);
              0, muf*v*(rf-hu), (kf + MARf_phi),                               bf,       (muf*g*hu) - ktf - (kf + MARf_phi_uf), 0;
              0, mur*v*(rr-hu), (kr + MARr_phi),                               br,       0,                                     (-mur*g*hu) - ktr - (kr + MARr_phi_ur);
              0, 0,             0,                                             -1,       0,                                     0];
              
    % Matriz H (Forças laterais como entradas)
    H_common = [ 1,   1;
                 lf, -lr;
                 0,   0;
                -rf,  0;
                 0,  -rr;
                 0,   0];
                 
    % Matriz G (Torques de controle como entradas)
    G = [ 0,  0;
          0,  0;
         -1, -1;
          0, -1;
         -1, -1;
          0,  0];
          
    % Cálculo do espaço de estados
    sys.A = -E_mat \ F_mat;
    B_passive = E_mat \ H_common;
    B_control = E_mat \ G;
    
    % A matriz B do Simulink espera [Fyf, Fyr, Tm_f, Tm_r]
    sys.B_total = [B_passive, B_control]; 
    sys.C = eye(6);
    sys.D = zeros(6, 4);
    
    % Metadata
    sys.states = {'beta', 'r', 'phi', 'phi_dot', 'phi_uf', 'phi_ur'};
    sys.inputs = {'Fyf', 'Fyr', 'Tm_f', 'Tm_r'};
    sys.velocity = v;
end

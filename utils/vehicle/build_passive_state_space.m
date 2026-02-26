function sys = build_passive_state_space(vehicle, MAR, v)
%BUILD_PASSIVE_STATE_SPACE Constroi o SS do Passivo apenas com [Fyf, Fyr]
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
    kAO_f = vehicle.kAO_f; kAO_r = vehicle.kAO_r;
    tAf = vehicle.tAf; tBf = vehicle.tBf; cf = vehicle.cf;
    tAr = vehicle.tAr; tBr = vehicle.tBr; cr = vehicle.cr;
    g = 9.81;
    
    % Momentos ARB Passiva 
    MARf_phi = (4 * kAO_f * ((tAf * tBf) / cf^2));
    MARr_phi = (4 * kAO_r * ((tAr * tBr) / cr^2));
    MARf_phi_uf = -(4 * kAO_f * (tAf^2 / cf^2));
    MARr_phi_ur = -(4 * kAO_r * (tAr^2 / cr^2));
    
    % Matrizes base
    E_mat = [ m*v,           0,   0,        -ms*h,             0,   0;
              0,             Izz, 0,        -Ixz,              0,   0;
              ms*v*h,        Ixz, -bf - br, -(Ixx + ms*(h^2)), bf,  br;
              muf*v*(rf-hu), 0,   bf,       0,                 -bf, 0;
              mur*v*(rr-hu), 0,   br,       0,                 0,   -br;
              0,             0,   1,        0,                 0,   0];
              
    F_mat = [ 0, m*v,           0,                                             0,        0,                                     0;
              0, 0,             0,                                             0,        0,                                     0;
              0, ms*v*h,        (ms*g*h) - (kf + MARf_phi) - (kr + MARr_phi), -bf - br, (kf + MARf_phi_uf),                    (kr + MARr_phi_ur);
              0, muf*v*(rf-hu), (kf + MARf_phi),                               bf,       (muf*g*hu) - ktf - (kf + MARf_phi_uf), 0;
              0, mur*v*(rr-hu), (kr + MARr_phi),                               br,       0,                                     (-mur*g*hu) - ktr - (kr + MARr_phi_ur); 
              0, 0,             0,                                             -1,       0,                                     0];
              
    % ===== MATRIZ H_COMMON CORRIGIDA PARA ENTRADA DO VOLANTE ====
    % O seu modelo espera o ângulo do volante (delta_f) e não as forças dos pneus 
    % já prontas, ou então a força dos pneus no seu bloco precisa do delta!
    % Mas vamos usar sua matriz original que depende das forças!
    
    H_common = [ 1,   1;
                 lf, -lr;
                 0,   0;
                -rf,  0;
                 0,  -rr;
                 0,   0];
                 
    sys.A = -E_mat \ F_mat;
    sys.B_total = E_mat \ H_common;
    sys.C = eye(6);
    sys.D = zeros(6, 2);
end

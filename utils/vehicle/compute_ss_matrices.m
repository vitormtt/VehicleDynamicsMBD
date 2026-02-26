function [A, B, C, D] = compute_ss_matrices(mode)
% Calcula matrizes State-Space 5DOF (Vu2016 + Khalil2018)
% mode: 'passive', 'pid', 'lqr'

if nargin < 1, mode = 'passive'; end

% Lê params do DD
dd = Simulink.data.dictionary.open(...
    'data/parameters/vehicle_params.sldd');
p = getValue(getEntry(getSection(dd, 'Design Data'), 'truck_params'));
dd.close();

% ARB moments (passive baseline)
MARf_phi = 4*p.kAO_f*(p.tAf^2)/p.cf^2;
MARr_phi = 4*p.kAO_r*(p.tAr^2)/p.cr^2;
MARf_phi_uf = -4*p.kAO_f*(p.tAf^2)/p.cf^2;
MARr_phi_ur = -4*p.kAO_r*(p.tAr^2)/p.cr^2;

% Matriz E (massa/inércia) - 6x6
E = [p.m*p.v          0                0              -p.ms*p.hcg     0       0;
    0                p.Izz            0              -p.Ixz          0       0;
    p.ms*p.v*p.hcg   p.Ixz           -(p.bf+p.br)   -p.Ixx-p.ms*p.hcg^2  p.bf  p.br;
    p.muf*p.v*(p.hu)  0               p.bf           0              -p.bf   0;
    p.mur*p.v*(p.hu)  0               p.br           0               0     -p.br;
    0                0                1              0               0      0];

% Matriz F (rigidez/amortecimento) - 6x6
F = [0  p.m*p.v  0  0  0  0;
    0  0        0  0  0  0;
    0  p.ms*p.v*p.hcg  (p.ms*9.81*p.hcg)-(p.kf+MARf_phi)-(p.kr+MARr_phi)  -(p.bf+p.br)  (p.kf+MARf_phi_uf)  (p.kr+MARr_phi_ur);
    0  p.muf*p.v*p.hu  (p.kf+MARf_phi)  p.bf  (p.muf*9.81*p.hu)-p.ktf-(p.kf+MARf_phi_uf)  0;
    0  p.mur*p.v*p.hu  (p.kr+MARr_phi)  p.br  0  (p.mur*9.81*p.hu)-p.ktr-(p.kr+MARr_phi_ur);
    0  0  0  -1  0  0];

% Matriz H (input: Fy_f, Fy_r) - 6x2
H = [1  1;
    p.lf  -p.lr;
    0  0;
    0  0;
    0  0;
    0  0];

% State-space
A = -E\F;  % 6x6
B = E\H;   % 6x2
C = eye(6);
D = zeros(6,2);

fprintf('✅ SS %s: A(6x6), B(6x2) | Eigenvalues: [', mode);
fprintf('%.2f ', real(eig(A))); fprintf(']\n');

% Salva no DD (opcional) - FIX
if nargout == 0
    dd = Simulink.data.dictionary.open(...
        'data/parameters/vehicle_params.sldd');
    dDataSect = getSection(dd, 'Design Data');

    % Remove se existir (R2024b)
    entries = {'A_passive', 'B_passive', 'C_passive', 'D_passive'};
    for i = 1:length(entries)
        try
            removeEntry(dDataSect, entries{i});
        catch
            % Não existe, ignora
        end
    end

    % Adiciona
    addEntry(dDataSect, 'A_passive', A);
    addEntry(dDataSect, 'B_passive', B);
    addEntry(dDataSect, 'C_passive', C);
    addEntry(dDataSect, 'D_passive', D);

    dd.saveChanges(); dd.close();
    fprintf('✅ A/B/C/D salvos no DD\n');
end


end

% Script para criar Data Dictionary com parametros de Gaspar(2004) para validacao 5-DOF
clear; clc;

data_dir = fullfile('data', 'parameters');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

dd_file = fullfile(data_dir, 'gaspar_params.sldd');

% Deleta se já existir para recriar do zero
if isfile(dd_file)
    delete(dd_file);
end

myDict = Simulink.data.dictionary.create(dd_file);
dataSect = getSection(myDict, 'Design Data');

%% Adicionando Parâmetros (Gaspar, 2004)
% --- Massa e Inercia ---
addEntry(dataSect, 'm_s', 12320);    % Sprung mass (kg)
addEntry(dataSect, 'm_uf', 555);     % Unsprung mass front (kg)
addEntry(dataSect, 'm_ur', 555);     % Unsprung mass rear (kg)
addEntry(dataSect, 'I_xx', 35000);   % Roll moment of inertia (kg.m^2)
addEntry(dataSect, 'I_zz', 42000);   % Yaw moment of inertia (kg.m^2)
addEntry(dataSect, 'I_xz', 0);       % Product of inertia (assumido 0 ou min) (kg.m^2)
addEntry(dataSect, 'h_s', 1.05);     % CG height of sprung mass from roll axis (m)
addEntry(dataSect, 'h_rcf', 0.6);    % Roll center height front (m)
addEntry(dataSect, 'h_rcr', 0.6);    % Roll center height rear (m)

% --- Geometria ---
addEntry(dataSect, 'a', 2.5);        % Distance from CG to front axle (m)
addEntry(dataSect, 'b', 3.5);        % Distance from CG to rear axle (m)
addEntry(dataSect, 'T_f', 2.0);      % Track width front (m)
addEntry(dataSect, 'T_r', 2.0);      % Track width rear (m)

% --- Pneus (Cornering stiffness) ---
addEntry(dataSect, 'C_f', 400000);   % Front cornering stiffness (N/rad) - 2x200k
addEntry(dataSect, 'C_r', 400000);   % Rear cornering stiffness (N/rad) - 2x200k

% --- Suspensão (Rigidez e Amortecimento Roll) ---
% Obs: Gaspar as vezes da K_f linear, aqui é torsional (K_phi_f = K_f * Tf^2 / 2)
addEntry(dataSect, 'k_phi_f', 500000); % Front suspension roll stiffness (Nm/rad)
addEntry(dataSect, 'k_phi_r', 500000); % Rear suspension roll stiffness (Nm/rad)
addEntry(dataSect, 'c_phi_f', 40000);  % Front suspension roll damping (Nms/rad)
addEntry(dataSect, 'c_phi_r', 40000);  % Rear suspension roll damping (Nms/rad)

% --- Pneus (Rigidez Vertical) ---
addEntry(dataSect, 'k_tf', 800000);  % Front tire vertical stiffness (N/m)
addEntry(dataSect, 'k_tr', 800000);  % Rear tire vertical stiffness (N/m)

% --- Variáveis Globais ---
addEntry(dataSect, 'g', 9.81);
addEntry(dataSect, 'v_x', 20);       % Velocidade padrao 20 m/s (72 km/h)

% --- Controle AARB ---
addEntry(dataSect, 'ARB_Mode', 0);   % 0: Passive, 1: LQR, 2: PID

saveChanges(myDict);
close(myDict);

fprintf('✅ Data Dictionary gaspar_params.sldd gerado com sucesso!\n');

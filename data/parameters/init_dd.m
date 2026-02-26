function init_dd()
%% init_dd.m - Inicialização completa do Data Dictionary
% Vitor  - UnB/PIBIC

%% Setup de paths
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));

% Adicionar paths de funções utilitárias
addpath(genpath(fullfile(project_root, 'utils')));
addpath(genpath(fullfile(project_root, 'scenarios')));

dd_path = fullfile(script_dir, 'vehicle_params.sldd');

% Verificar se DD está aberto
open_dicts = Simulink.data.dictionary.getOpenDictionaryPaths;
if any(contains(open_dicts, 'vehicle_params.sldd'))
    Simulink.data.dictionary.closeAll('-discard'); 
    pause(0.5); 
end

%% Abrir/criar Data Dictionary
if ~isfile(dd_path)
    Simulink.data.dictionary.create(dd_path);
end

dd = Simulink.data.dictionary.open(dd_path);
dDataSect = getSection(dd, 'Design Data');

%% 1. Carregar parâmetros do veículo (função externa)
vehicle = load_vehicle_params('truck'); % Função separada
update_or_add(dDataSect, 'vehicle', vehicle);

%% 2. Calcular momentos ARB passiva
MAR_f = calculate_passive_arb(vehicle.kAO_f, vehicle.tAf, ...
    vehicle.tBf, vehicle.cf);
MAR_r = calculate_passive_arb(vehicle.kAO_r, vehicle.tAr, ...
    vehicle.tBr, vehicle.cr);

MAR.f = MAR_f;
MAR.r = MAR_r;
update_or_add(dDataSect, 'MAR', MAR);

%% 3. Construir espaço de estados 
v = 70 / 3.6; % Velocidade padrão

% ATIVO (4 entradas)
sys_ativo = build_active_state_space(vehicle, MAR, v);
A_ativo = Simulink.Parameter; A_ativo.Value = sys_ativo.A; A_ativo.StorageClass = 'Auto';
B_ativo = Simulink.Parameter; B_ativo.Value = sys_ativo.B_total; B_ativo.StorageClass = 'Auto';
C_ativo = Simulink.Parameter; C_ativo.Value = sys_ativo.C; C_ativo.StorageClass = 'Auto';
D_ativo = Simulink.Parameter; D_ativo.Value = sys_ativo.D; D_ativo.StorageClass = 'Auto';
update_or_add(dDataSect, 'A_ativo', A_ativo); update_or_add(dDataSect, 'B_ativo', B_ativo);
update_or_add(dDataSect, 'C_ativo', C_ativo); update_or_add(dDataSect, 'D_ativo', D_ativo);

% PASSIVO (2 entradas)
sys_passive = build_passive_state_space(vehicle, MAR, v);
A_passive = Simulink.Parameter; A_passive.Value = sys_passive.A; A_passive.StorageClass = 'Auto';
B_passive = Simulink.Parameter; B_passive.Value = sys_passive.B_total; B_passive.StorageClass = 'Auto';
C_passive = Simulink.Parameter; C_passive.Value = sys_passive.C; C_passive.StorageClass = 'Auto';
D_passive = Simulink.Parameter; D_passive.Value = sys_passive.D; D_passive.StorageClass = 'Auto';
update_or_add(dDataSect, 'A_passive', A_passive); update_or_add(dDataSect, 'B_passive', B_passive);
update_or_add(dDataSect, 'C_passive', C_passive); update_or_add(dDataSect, 'D_passive', D_passive);

%% 4. Parâmetros de controle
% Variant control
ARB_Mode = Simulink.Parameter(2); % Default: PID
ARB_Mode.StorageClass = 'Auto'; % ← MUDAR de 'ExportedGlobal'
update_or_add(dDataSect, 'ARB_Mode', ARB_Mode);

% PID
PID_params = struct();
PID_params.Kp = 23.15;
PID_params.Ki = 10.2;
PID_params.Kd = 5.4;
PID_params.Tm_max = 15000; % N·m
update_or_add(dDataSect, 'PID_params', PID_params);

% LQR
lqr_file = fullfile(project_root, 'controllers', 'lqr_gains_v70.mat');
if isfile(lqr_file)
    load(lqr_file, 'K_r');
    LQR_params.K = K_r;
else
    LQR_params.K = zeros(2, 6); % Corrigido para 6 estados
    LQR_params.Q = diag([1e10, 5e7, 2e7, 2e7]);
    LQR_params.R = diag([1e-2, 1e-2]);
    LQR_params.T = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
end
update_or_add(dDataSect, 'LQR_params', LQR_params);

%% 5. Salvar
saveChanges(dd);
close(dd);

end

%% Função auxiliar
function update_or_add(dDataSect, var_name, var_value)
if ~isempty(find(dDataSect, 'Name', var_name))
    assignin(dDataSect, var_name, var_value);
else
    addEntry(dDataSect, var_name, var_value);
end
end

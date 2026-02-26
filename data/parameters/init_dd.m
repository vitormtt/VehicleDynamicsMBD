function init_dd()
%% init_dd.m - Inicialização completa do Data Dictionary
% Vitor  - UnB/PIBIC

%% Setup de paths
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));

% Adicionar paths de funções utilitárias
addpath(genpath(fullfile(project_root, 'utils')));
addpath(genpath(fullfile(project_root, 'scenarios')));

%% ═══════════════════════════════════════════════════════
%% NOVO: Fechar DD se já estiver aberto
%% ═══════════════════════════════════════════════════════
dd_path = fullfile(script_dir, 'vehicle_params.sldd');

% Verificar se DD está aberto
open_dicts = Simulink.data.dictionary.getOpenDictionaryPaths;
if any(contains(open_dicts, 'vehicle_params.sldd'))
    fprintf('⚠️  DD já aberto. Fechando...\n');
    Simulink.data.dictionary.closeAll('-discard'); % Fechar sem salvar
    pause(0.5); % Aguardar fechamento
end

%% Abrir/criar Data Dictionary
if ~isfile(dd_path)
    fprintf('⚠️  DD não encontrado. Criando novo...\n');
    Simulink.data.dictionary.create(dd_path);
end

dd = Simulink.data.dictionary.open(dd_path);
dDataSect = getSection(dd, 'Design Data');

fprintf('═══ Inicializando Data Dictionary ═══\n\n');

%% 1. Carregar parâmetros do veículo (função externa)
fprintf('1️⃣  Carregando parâmetros do veículo...\n');
vehicle = load_vehicle_params('truck'); % Função separada

update_or_add(dDataSect, 'vehicle', vehicle);
fprintf('   ✅ %d parâmetros carregados (%s)\n\n', ...
    numel(fieldnames(vehicle)), vehicle.name);

%% 2. Calcular momentos ARB passiva
fprintf('2️⃣  Calculando momentos ARB passiva...\n');
MAR_f = calculate_passive_arb(vehicle.kAO_f, vehicle.tAf, ...
    vehicle.tBf, vehicle.cf);
MAR_r = calculate_passive_arb(vehicle.kAO_r, vehicle.tAr, ...
    vehicle.tBr, vehicle.cr);

MAR.f = MAR_f;
MAR.r = MAR_r;
update_or_add(dDataSect, 'MAR', MAR);

fprintf('   Front: φ=%.1f, φ_u=%.1f Nm/rad\n', MAR_f.phi, MAR_f.phi_u);
fprintf('   Rear:  φ=%.1f, φ_u=%.1f Nm/rad\n', MAR_r.phi, MAR_r.phi_u);
fprintf('   ✅ Momentos calculados\n\n');

%% 3. Construir espaço de estados ativo (SUBSTITUIR LINHAS ~60-90)
fprintf('3️⃣  Construindo espaço de estados (5-DOF)...\n');
v = 70 / 3.6; % Velocidade padrão
sys = build_active_state_space(vehicle, MAR, v);

% Criar Simulink.Parameter com StorageClass = 'Auto' (não 'ExportedGlobal')
A_ativo = Simulink.Parameter;
A_ativo.Value = sys.A;
A_ativo.StorageClass = 'Auto'; % ← CRÍTICO: mudança de ExportedGlobal
A_ativo.Description = sprintf('State matrix (6x6) @ %.1f km/h', v*3.6);

B_ativo = Simulink.Parameter;
B_ativo.Value = sys.B_total;
B_ativo.StorageClass = 'Auto'; % ← CRÍTICO
B_ativo.Description = 'Input matrix (6x4): [δf, δr, Tf, Tr]';

C_ativo = Simulink.Parameter;
C_ativo.Value = eye(6);
C_ativo.StorageClass = 'Auto'; % ← CRÍTICO
C_ativo.Description = 'Output matrix (identity)';

D_ativo = Simulink.Parameter;
D_ativo.Value = zeros(6, 4);
D_ativo.StorageClass = 'Auto'; % ← CRÍTICO
D_ativo.Description = 'Feedthrough matrix (zeros)';

update_or_add(dDataSect, 'A_ativo', A_ativo);
update_or_add(dDataSect, 'B_ativo', B_ativo);
update_or_add(dDataSect, 'C_ativo', C_ativo);
update_or_add(dDataSect, 'D_ativo', D_ativo);

fprintf('   A: %dx%d, B: %dx%d, C: %dx%d, D: %dx%d\n', ...
        size(sys.A), size(sys.B_total), 6, 6, 6, 4);
fprintf('   ✅ Matrizes configuradas\n\n');


%% 4. Parâmetros de controle (LINHA ~90 do init_dd.m)
fprintf('4️⃣  Configurando controladores...\n');

% Variant control
ARB_Mode = Simulink.Parameter(2); % Default: PID
ARB_Mode.StorageClass = 'Auto'; % ← MUDAR de 'ExportedGlobal'
ARB_Mode.Description = '1=Passive, 2=PID, 3=MPC';
update_or_add(dDataSect, 'ARB_Mode', ARB_Mode);


% PID
PID_params = struct();
PID_params.Kp = 23.15;
PID_params.Ki = 10.2;
PID_params.Kd = 5.4;
PID_params.Tm_max = 15000; % N·m
update_or_add(dDataSect, 'PID_params', PID_params);
fprintf('   PID: Kp=%.2f, Ki=%.1f, Kd=%.1f\n', ...
    PID_params.Kp, PID_params.Ki, PID_params.Kd);

% LQR
lqr_file = fullfile(project_root, 'controllers', 'lqr_gains_v70.mat');
if isfile(lqr_file)
    load(lqr_file, 'K_r');
    LQR_params.K = K_r;
    fprintf('   LQR: K_r carregado (2x4)\n');
else
    LQR_params.K = zeros(2, 4);
    LQR_params.Q = diag([1e10, 5e7, 2e7, 2e7]);
    LQR_params.R = diag([1e-2, 1e-2]);
    LQR_params.T = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
    fprintf('   LQR: Placeholder (execute design_lqr_controller.m)\n');
end
update_or_add(dDataSect, 'LQR_params', LQR_params);
fprintf('   ✅ Controladores configurados\n\n');

%% 5. Salvar
saveChanges(dd);
close(dd);

fprintf('✅ Data Dictionary inicializado!\n');
fprintf('   Localização: %s\n', dd_path);
end

%% Função auxiliar
function update_or_add(dDataSect, var_name, var_value)
if ~isempty(find(dDataSect, 'Name', var_name))
    assignin(dDataSect, var_name, var_value);
else
    addEntry(dDataSect, var_name, var_value);
end
end

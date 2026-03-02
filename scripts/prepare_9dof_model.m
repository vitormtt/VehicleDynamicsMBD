function prepare_9dof_model()
% PREPARE_9DOF_MODEL Cria e pré-configura o modelo 9-DOF vazio
%
% Este script:
% 1. Cria o arquivo models/variants/9dof/vehicle_9dof_model.slx
% 2. Vincula automaticamente o Data Dictionary (vehicle_params.sldd)
% 3. Configura o Solver para ode45 (variável)

disp('🛠️ Preparando scaffolding para o modelo 9-DOF...');

% Diretório de destino
model_dir = fullfile(pwd, 'models', 'variants', '9dof');
if ~isfolder(model_dir)
    mkdir(model_dir);
end

model_name = 'vehicle_9dof_model';
model_path = fullfile(model_dir, [model_name, '.slx']);

% Verifica se já existe para não sobscrever
if isfile(model_path)
    warning('O modelo %s já existe. Nenhuma alteração foi feita.', model_name);
    return;
end

% 1. Cria um modelo vazio e salva
new_sys = new_system(model_name);
save_system(new_sys, model_path);

% 2. Vincula o Data Dictionary (A forma MAIS CORRETA do Simulink)
% O DD deve estar na raiz do projeto ou ser acessível no Path
dd_name = 'vehicle_params.sldd';
set_param(model_name, 'DataDictionary', dd_name);
fprintf('   ✅ Data Dictionary (%s) vinculado ao modelo.\n', dd_name);

% 3. Configurações de Solver recomendadas para MBD
set_param(model_name, 'SolverType', 'Variable-step');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'RelTol', '1e-4');
set_param(model_name, 'MaxStep', '0.01'); % Evita passos muito longos em transientes rápidos
fprintf('   ✅ Solver configurado para ode45 (Variable-step).\n');

% 4. Salva e abre para edição
save_system(new_sys);
open_system(new_sys);

disp('✅ Modelo 9-DOF criado e pronto para a modelagem visual!');
end
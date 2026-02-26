function setup_environment()
%SETUP_ENVIRONMENT Configura o ambiente MATLAB para o projeto VehicleDynamicsMBD
%   Adiciona todas as pastas necessarias ao path do MATLAB e 
%   configura os diretorios de build e cache do Simulink para 
%   evitar poluir o repositorio com arquivos gerados.
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 26/02/2026

    disp('=======================================================');
    disp('  Inicializando Ambiente: VehicleDynamicsMBD (14-DOF)  ');
    disp('=======================================================');

    %% 1. Configurar o Path do Projeto
    project_root = pwd;
    disp(['Diretorio Raiz: ', project_root]);
    
    % Pastas a serem incluidas no Path (excluindo .git, build, cache)
    folders_to_add = {...
        'controllers', ...
        'data', ...
        'data/parameters', ...
        'docs', ...
        'models', ...
        'models/variants', ...
        'models/variants/5dof', ...
        'models/variants/9dof', ...
        'models/variants/14dof', ...
        'models/components', ...
        'results', ...
        'scenarios', ...
        'utils', ...
        'utils/simulation', ...
        'utils/vehicle', ...
        'validation' ...
    };

    added_count = 0;
    for i = 1:length(folders_to_add)
        folder_path = fullfile(project_root, folders_to_add{i});
        if exist(folder_path, 'dir')
            addpath(genpath(folder_path));
            added_count = added_count + 1;
        end
    end
    
    disp(['✅ ', num2str(added_count), ' diretorios adicionados ao MATLAB Path.']);
    
    %% 2. Configurar Simulink Cache e CodeGen
    cache_dir = fullfile(project_root, 'build', 'cache');
    codegen_dir = fullfile(project_root, 'build', 'codegen');
    
    if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end
    if ~exist(codegen_dir, 'dir'), mkdir(codegen_dir); end
    
    Simulink.fileGenControl('set', ...
        'CacheFolder', cache_dir, ...
        'CodeGenFolder', codegen_dir, ...
        'KeepPreviousPath', false, ...
        'CreateDir', true);
        
    disp('✅ Diretorios slprj redirecionados para /build/.');
    
    %% 3. Inicializar Data Dictionary (se necessario)
    try
        if exist('init_dd', 'file')
            % Descomente abaixo se quiser rodar init_dd() automaticamente
            % init_dd(); 
            disp('✅ Data Dictionary disponivel no Path.');
        end
    catch ME
        warning('Falha ao verificar Data Dictionary: %s', ME.message);
    end
    
    disp('=======================================================');
    disp('Ambiente pronto! Voce pode rodar as simulacoes agora.');
    disp('=======================================================');
end

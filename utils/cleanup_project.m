function cleanup_project()
%CLEANUP_PROJECT Remove arquivos duplicados e reorganiza estrutura
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    fprintf('═══ Limpeza e Reorganização do Projeto ═══\n\n');
    
    %% 1. Criar nova estrutura de pastas
    fprintf('1️⃣  Criando estrutura de diretórios...\n');
    folders = {
        'models/variants/5dof', ...
        'models/variants/9dof', ...
        'models/variants/14dof', ...
        'models/components/tire', ...
        'models/components/suspension', ...
        'models/components/chassis', ...
        'utils/vehicle', ...
        'utils/simulation', ...
        'utils/validation', ...
        'scenarios', ...
        'controllers', ...
        'results/5dof', ...
        'results/9dof', ...
        'results/14dof'
    };
    
    for i = 1:length(folders)
        if ~exist(folders{i}, 'dir')
            mkdir(folders{i});
            fprintf('   ✅ Criado: %s\n', folders{i});
        end
    end
    fprintf('\n');
    
    %% 2. Mover modelos legados para 5dof/
    fprintf('2️⃣  Migrando modelos para 5dof/...\n');
    
    legacy_models = {
        'models/variants/3dof_truck/Truck_5DOF_ARB.slx', ...
        'models/variants/3dof/ARB_ACTIVE_PID.slx', ...
        'models/variants/3dof/ARB_ACTIVE_LQR.slx', ...
        'models/variants/3dof/ARB_PASSIVE.slx'
    };
    
    for i = 1:length(legacy_models)
        if exist(legacy_models{i}, 'file')
            [~, name, ext] = fileparts(legacy_models{i});
            dest = fullfile('models/variants/5dof', [name, ext]);
            
            if ~exist(dest, 'file')
                copyfile(legacy_models{i}, dest);
                fprintf('   📦 Copiado: %s → 5dof/\n', name);
            else
                fprintf('   ⚠️  Já existe: %s\n', name);
            end
        end
    end
    fprintf('\n');
    
    %% 3. Mover funções para utils/
    fprintf('3️⃣  Reorganizando utilitários...\n');
    
    util_moves = {
        'utils/compute_ss_matrices.m', 'utils/vehicle/';
        'utils/build_truck_model.m', 'utils/vehicle/';
        'models/components/tire/pacejkaLateralForce.m', 'utils/vehicle/'
    };
    
    for i = 1:size(util_moves, 1)
        src = util_moves{i, 1};
        dest_dir = util_moves{i, 2};
        
        if exist(src, 'file')
            [~, name, ext] = fileparts(src);
            dest = fullfile(dest_dir, [name, ext]);
            
            if ~exist(dest, 'file')
                movefile(src, dest);
                fprintf('   ✅ Movido: %s → %s\n', name, dest_dir);
            end
        end
    end
    fprintf('\n');
    
    %% 4. Deletar duplicatas e arquivos temporários
    fprintf('4️⃣  Removendo arquivos desnecessários...\n');
    
    % Padrões de arquivos para deletar
    delete_patterns = {
        '**/*Copy*.m', ...
        '**/*Copy*.slx', ...
        '**/*.slxc', ...
        '**/*.autosave', ...
        '**/*.r2020a', ...
        'models/variants/3dof', ...
        'models/variants/3dof_truck'
    };
    
    deleted_count = 0;
    for i = 1:length(delete_patterns)
        pattern = delete_patterns{i};
        
        if contains(pattern, '**')
            % Usar dir recursivo para padrões
            files = dir(pattern);
            for j = 1:length(files)
                if ~files(j).isdir
                    delete(fullfile(files(j).folder, files(j).name));
                    deleted_count = deleted_count + 1;
                end
            end
        else
            % Deletar pasta diretamente
            if exist(pattern, 'dir')
                rmdir(pattern, 's');
                fprintf('   🗑️  Pasta removida: %s\n', pattern);
            end
        end
    end
    fprintf('   ✅ %d arquivos temporários removidos\n\n', deleted_count);
    
    %% 5. Mover scripts de dados para controllers/
    fprintf('5️⃣  Organizando scripts de design...\n');
    
    data_scripts = dir('models/variants/5dof/script_and_data_*.mlx');
    for i = 1:length(data_scripts)
        src = fullfile(data_scripts(i).folder, data_scripts(i).name);
        dest = fullfile('controllers', data_scripts(i).name);
        
        if ~exist(dest, 'file')
            movefile(src, dest);
            fprintf('   ✅ Movido: %s → controllers/\n', data_scripts(i).name);
        end
    end
    fprintf('\n');
    
    %% 6. Atualizar referências no DD
    fprintf('6️⃣  Atualizando caminhos no Data Dictionary...\n');
    dd_path = 'data/parameters/vehicle_params.sldd';
    
    if exist(dd_path, 'file')
        % Recriar DD limpo
        delete(dd_path);
        fprintf('   ✅ DD antigo removido, execute init_dd() para recriar\n');
    end
    fprintf('\n');
    
    %% 7. Gerar relatório atualizado
    fprintf('7️⃣  Gerando relatório atualizado...\n');
    generate_project_report();
    
    fprintf('\n╔═══════════════════════════════════════════════════════╗\n');
    fprintf('║          LIMPEZA CONCLUÍDA COM SUCESSO ✅            ║\n');
    fprintf('╚═══════════════════════════════════════════════════════╝\n');
    fprintf('\nPróximos passos:\n');
    fprintf('1. Execute: cd(''data/parameters''); init_dd();\n');
    fprintf('2. Crie manualmente: models/variants/5dof/5dof_model.slx\n');
    fprintf('3. Teste: run_5dof_simulation(''PID'', ''DLC'', 70);\n\n');
end

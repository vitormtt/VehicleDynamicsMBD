function sync_matlab_project()
% SYNC_MATLAB_PROJECT Sincroniza os arquivos reais com o arquivo .prj
%
% Este script:
% 1. Encontra arquivos deletados/movidos e os remove do projeto (.prj).
% 2. Encontra novos arquivos válidos e os adiciona ao projeto.
% 3. Ignora pastas de cache, resultados e build.
%
% Autor: Vitor Yukio
% Data: 01/03/2026

disp('🔄 Iniciando sincronização automática do MATLAB Project...');

%% 1. Tentar obter o objeto do projeto atual
try
    proj = currentProject;
catch
    warning('Nenhum Simulink Project aberto. Tentando carregar da raiz...');
    try
        proj = matlab.project.loadProject(pwd);
    catch
        error('Falha ao carregar o projeto. Certifique-se de estar na pasta raiz.');
    end
end

%% 2. Remover arquivos "Missing" (X vermelho)
projectFiles = proj.Files;
missingCount = 0;

for i = 1:length(projectFiles)
    fileObj = projectFiles(i);
    % Se o arquivo físico não existe mais, remove do projeto
    if ~isfile(fileObj.Path) && ~isfolder(fileObj.Path)
        removeFile(proj, fileObj.Path);
        missingCount = missingCount + 1;
        fprintf('   🗑️ Removido: %s\n', fileObj.Path);
    end
end
if missingCount == 0
    disp('   ✅ Nenhum arquivo ausente encontrado.');
end

%% 3. Adicionar arquivos novos
% Definir pastas a serem escaneadas (ignora build, slprj, results, resources, cache)
foldersToScan = {'data', 'models', 'utils', 'validation', 'controllers', 'scenarios', 'docs', 'scripts'};
addedCount = 0;

for i = 1:length(foldersToScan)
    folderPath = fullfile(proj.RootFolder, foldersToScan{i});
    if isfolder(folderPath)
        % Busca todos os arquivos iterativamente nas subpastas
        files = dir(fullfile(folderPath, '**', '*.*'));
        
        for j = 1:length(files)
            if ~files(j).isdir
                filePath = fullfile(files(j).folder, files(j).name);
                
                % Ignora arquivos de sistema e temporários
                if endsWith(files(j).name, '~') || startsWith(files(j).name, '.') || endsWith(files(j).name, '.asv')
                    continue;
                end
                
                % Tenta adicionar ao projeto (se já existir, a API ignora sem erro grave)
                try
                    % Verifica se o arquivo já está no projeto
                    isFileInProject = any(strcmp({projectFiles.Path}, filePath));
                    if ~isFileInProject
                        addFile(proj, filePath);
                        addedCount = addedCount + 1;
                        fprintf('   ➕ Adicionado: %s\n', filePath);
                    end
                catch
                    % Ignora falhas silenciosas
                end
            end
        end
    end
end

% Adiciona scripts soltos na raiz (como startup.m)
rootFiles = dir(fullfile(proj.RootFolder, '*.m'));
for i = 1:length(rootFiles)
    filePath = fullfile(rootFiles(i).folder, rootFiles(i).name);
    try
        isFileInProject = any(strcmp({projectFiles.Path}, filePath));
        if ~isFileInProject
            addFile(proj, filePath);
            addedCount = addedCount + 1;
            fprintf('   ➕ Adicionado: %s\n', filePath);
        end
    catch
    end
end

if addedCount == 0
    disp('   ✅ Nenhum arquivo novo para adicionar.');
end

disp('✅ Sincronização concluída com sucesso!');
end
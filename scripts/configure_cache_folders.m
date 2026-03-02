function configure_cache_folders()
% CONFIGURE_CACHE_FOLDERS Redireciona saídas de build/cache do Simulink
%
% Este script configura o projeto para que as pastas slprj, cache e build
% não fiquem espalhadas pelo projeto, sendo roteadas para a pasta \cache.
%
% Autor: Vitor Yukio

disp('🔧 Configurando pastas de cache e build do Simulink...');

try
    proj = currentProject;
catch
    try
        proj = matlab.project.loadProject(pwd);
    catch
        error('Falha ao carregar o projeto. Execute na raiz do projeto.');
    end
end

% Diretório alvo para lixo do Simulink
cacheFolder = fullfile(proj.RootFolder, 'cache');
if ~isfolder(cacheFolder)
    mkdir(cacheFolder);
end

% 1. Configurar pasta de Cache do Simulink (Simulink Cache files .slxc)
proj.SimulinkCacheFolder = cacheFolder;
fprintf('   ✅ Simulink Cache folder configurado para: %s\n', cacheFolder);

% 2. Configurar pasta de Build (Código gerado, slprj)
proj.SimulinkCodeGenFolder = cacheFolder;
fprintf('   ✅ Code Generation folder configurado para: %s\n', cacheFolder);

% 3. Redirecionar slprj especificamente (redundância global)
Simulink.fileGenControl('set', ...
    'CacheFolder', cacheFolder, ...
    'CodeGenFolder', cacheFolder, ...
    'KeepPreviousPath', false, ...
    'CreateDir', true);
fprintf('   ✅ fileGenControl roteado para o cache.\n');

disp('✅ Configuração concluída! (Arquivos slprj soltos podem ser deletados agora).');
end
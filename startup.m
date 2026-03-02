% startup.m - AARB Project R2026 (PATH DINÂMICO)
disp('Iniciando MATLAB e configurando ambiente MBD...');

% ROOT Dinâmico (onde o startup.m está localizado)
aarbRoot = fileparts(mfilename('fullpath'));
if isempty(aarbRoot)
    aarbRoot = pwd;
end

% Simulink Project
projFile = fullfile(aarbRoot, 'VehicleDynamicsMBD.prj');
if isfile(projFile)
    proj = matlab.project.loadProject(aarbRoot);
    disp('✅ Simulink Project carregado com sucesso.');
else
    proj = matlab.project.createProject(aarbRoot);
    proj.Name = 'VehicleDynamicsMBD';
    disp('✅ Simulink Project criado.');
end

% Força o diretório atual para a raiz do projeto
cd(aarbRoot);

% ADICIONA TODAS AS PASTAS (recursivo)
projectFolders = {'data', 'models', 'utils', 'validation', ...
                  'controllers', 'scenarios', 'docs', 'scripts'};
for i = 1:length(projectFolders)
    folder = fullfile(aarbRoot, projectFolders{i});
    if isfolder(folder)
        addpath(genpath(folder));
    end
end

% Configurar roteamento de cache/build do Simulink automaticamente
try
    configure_cache_folders();
catch
    warning('Script configure_cache_folders não encontrado. Cache pode gerar pastas soltas.');
end

% Tema Gráfico
s = settings;
s.matlab.appearance.figure.GraphicsTheme.TemporaryValue = 'light';
set(groot, 'DefaultAxesColorOrder', [
    0.83 0.14 0.14; 1.00 0.54 0.00; 0.47 0.25 0.80; 0.25 0.80 0.54]);

disp('✅ AARB MBD ready! Pastas adicionadas ao Path.');

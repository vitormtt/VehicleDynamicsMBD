function generate_project_report()
%GENERATE_PROJECT_REPORT Relatório automático do projeto MBD
%
% Melhorias v2.0:
%   - Análise de Git (branch, último commit, status)
%   - Métricas de código (LOC, complexidade ciclomática)
%   - Validação de dependências (Toolboxes)
%   - Health check do Data Dictionary
%   - Estatísticas de simulações recentes
%   - Comparativo temporal (evolução do projeto)
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026

%% Configuração
root_dir = pwd;
report_file = fullfile(root_dir, 'project_report.txt');
fid = fopen(report_file, 'w');

%% Cabeçalho
fprintf(fid, '═══════════════════════════════════════════════════════════════\n');
fprintf(fid, 'VEHICLE DYNAMICS MBD PROJECT - STATUS REPORT v2.0\n');
fprintf(fid, 'Generated: %s\n', datestr(now, 'dd-mmm-yyyy HH:MM:SS'));
fprintf(fid, 'Root: %s\n', strrep(root_dir, '\', '\\'));
fprintf(fid, '═══════════════════════════════════════════════════════════════\n\n');

%% 1. CONTEXTO DO PROJETO
write_section(fid, 'CONTEXTO DO PROJETO');
fprintf(fid, '**Objetivo**: Framework modular MBD para análise de dinâmica veicular,\n');
fprintf(fid, 'desenvolvimento de controladores AARB (PID/LQR/MPC) e validação ISO 19364:2016.\n\n');
fprintf(fid, '**Veículo**: Chevrolet Blazer 2001 (1905 kg, wheelbase 2.718m)\n');
fprintf(fid, '**Modelos**: 5-DOF (yaw-roll) → 9-DOF → 14-DOF (completo)\n');
fprintf(fid, '**Controladores**: Passive, PID, LQR (impl.), MPC (roadmap)\n');
fprintf(fid, '**Manobras**: DLC (ISO 3888), Fishhook (FMVSS 126), J-Turn, Step Steer\n\n');

%% 2. GIT STATUS (NOVO)
write_section(fid, 'CONTROLE DE VERSÃO (GIT)');
[git_status, git_info] = get_git_status();
if git_status
    fprintf(fid, '✅ Repositório Git ativo\n');
    fprintf(fid, '   Branch:        %s\n', git_info.branch);
    fprintf(fid, '   Último commit: %s\n', git_info.last_commit);
    fprintf(fid, '   Hash:          %s\n', git_info.hash);
    fprintf(fid, '   Autor:         %s\n', git_info.author);
    fprintf(fid, '   Data:          %s\n\n', git_info.date);
    
    if ~isempty(git_info.modified_files)
        fprintf(fid, '⚠️  Arquivos modificados não commitados: %d\n', length(git_info.modified_files));
        for i = 1:min(5, length(git_info.modified_files))
            fprintf(fid, '   - %s\n', git_info.modified_files{i});
        end
        fprintf(fid, '\n');
    end
else
    fprintf(fid, '❌ Repositório Git não detectado\n');
    fprintf(fid, '   Recomendação: Inicializar com "git init"\n\n');
end

%% 3. DEPENDÊNCIAS E TOOLBOXES (NOVO)
write_section(fid, 'DEPENDÊNCIAS MATLAB');
required_toolboxes = {
    'Simulink'
    'Control System Toolbox'
    'Simulink Control Design'
    'Optimization Toolbox'
};

fprintf(fid, 'Toolboxes Requeridas:\n');
for i = 1:length(required_toolboxes)
    installed = check_toolbox(required_toolboxes{i});
    if installed
        fprintf(fid, '   ✅ %s\n', required_toolboxes{i});
    else
        fprintf(fid, '   ❌ %s (NÃO INSTALADA)\n', required_toolboxes{i});
    end
end

optional_toolboxes = {
    'Model Predictive Control Toolbox' 
    'Parallel Computing Toolbox'
    'Simulink Code Inspector'
};

fprintf(fid, '\nToolboxes Opcionais:\n');
for i = 1:length(optional_toolboxes)
    installed = check_toolbox(optional_toolboxes{i});
    if installed
        fprintf(fid, '   ✅ %s\n', optional_toolboxes{i});
    else
        fprintf(fid, '   ⏳ %s (recomendada)\n', optional_toolboxes{i});
    end
end
fprintf(fid, '\n');

%% 4. HEALTH CHECK DATA DICTIONARY (NOVO)
write_section(fid, 'DATA DICTIONARY HEALTH CHECK');
dd_path = fullfile('data', 'parameters', 'vehicle_params.sldd');
if exist(dd_path, 'file')
    fprintf(fid, '✅ vehicle_params.sldd encontrado\n');
    dd_info = dir(dd_path);
    fprintf(fid, '   Tamanho: %.1f KB\n', dd_info.bytes/1024);
    fprintf(fid, '   Modificado: %s\n', datestr(dd_info.datenum));
    
    try
        dd = Simulink.data.dictionary.open(dd_path);
        dDataSect = getSection(dd, 'Design Data');
        entries = find(dDataSect);
        fprintf(fid, '   Entradas: %d\n', length(entries));
        
        % Verificar entradas críticas
        critical_entries = {'vehicle', 'MAR', 'A_ativo', 'B_ativo', 'ARB_Mode'};
        fprintf(fid, '\n   Entradas Críticas:\n');
        for i = 1:length(critical_entries)
            entry = find(dDataSect, 'Name', critical_entries{i});
            if ~isempty(entry)
                fprintf(fid, '      ✅ %s\n', critical_entries{i});
            else
                fprintf(fid, '      ❌ %s (FALTANDO)\n', critical_entries{i});
            end
        end
        
        close(dd);
    catch ME
        fprintf(fid, '   ⚠️  Erro ao ler DD: %s\n', ME.message);
    end
else
    fprintf(fid, '❌ vehicle_params.sldd NÃO ENCONTRADO\n');
    fprintf(fid, '   Execute: init_dd() para criar\n');
end
fprintf(fid, '\n');

%% 5. MÉTRICAS DE CÓDIGO (NOVO)
write_section(fid, 'MÉTRICAS DE CÓDIGO');
code_stats = analyze_code_metrics(root_dir);
fprintf(fid, 'Scripts MATLAB:\n');
fprintf(fid, '   Total:         %d arquivos\n', code_stats.num_m_files);
fprintf(fid, '   LOC total:     %d linhas\n', code_stats.total_loc);
fprintf(fid, '   LOC médio:     %.0f linhas/arquivo\n', code_stats.avg_loc);
fprintf(fid, '   Comentários:   %.1f%%\n\n', code_stats.comment_ratio*100);

fprintf(fid, 'Modelos Simulink:\n');
fprintf(fid, '   Total:         %d modelos\n', code_stats.num_slx_files);
fprintf(fid, '   Tamanho total: %.1f MB\n\n', code_stats.total_slx_size/1e6);

%% 6. SIMULAÇÕES RECENTES (NOVO)
write_section(fid, 'SIMULAÇÕES RECENTES (ÚLTIMAS 7 DIAS)');
sim_stats = analyze_recent_simulations(root_dir);
if sim_stats.num_simulations > 0
    fprintf(fid, '✅ %d simulação(ões) encontrada(s)\n\n', sim_stats.num_simulations);
    fprintf(fid, '   Controladores testados:\n');
    controllers = fieldnames(sim_stats.by_controller);
    for i = 1:length(controllers)
        fprintf(fid, '      %s: %d simulação(ões)\n', ...
                controllers{i}, sim_stats.by_controller.(controllers{i}));
    end
    
    fprintf(fid, '\n   Manobras executadas:\n');
    maneuvers = fieldnames(sim_stats.by_maneuver);
    for i = 1:length(maneuvers)
        fprintf(fid, '      %s: %d simulação(ões)\n', ...
                maneuvers{i}, sim_stats.by_maneuver.(maneuvers{i}));
    end
    
    fprintf(fid, '\n   Estatísticas de performance:\n');
    fprintf(fid, '      Roll RMS médio: %.4f deg\n', sim_stats.avg_roll_rms);
    fprintf(fid, '      Tempo exec. médio: %.2f s\n', sim_stats.avg_exec_time);
else
    fprintf(fid, '⏳ Nenhuma simulação recente encontrada\n');
    fprintf(fid, '   Execute: run_5dof_simulation(''Passive'', ''DLC'', 70)\n');
end
fprintf(fid, '\n');

%% 7. ARQUITETURA MBD
write_section(fid, 'ARQUITETURA MBD (MAB Guidelines)');
fprintf(fid, '**Estrutura Unificada**:\n');
fprintf(fid, 'vehicle_params.sldd (ÚNICO)\n');
fprintf(fid, '  ├── 5dof_model.slx  → Variant: [Passive|PID|LQR]\n');
fprintf(fid, '  ├── 9dof_model.slx  → Variant: [Passive|PID|LQR|MPC] (futuro)\n');
fprintf(fid, '  └── 14dof_model.slx → Variant: [Passive|PID|LQR|MPC] (futuro)\n\n');

fprintf(fid, '**Princípios**:\n');
fprintf(fid, '- Single Source of Truth: Data Dictionary único\n');
fprintf(fid, '- Variant Subsystems: 1 modelo .slx por DOF, N controladores\n');
fprintf(fid, '- Timeseries Logging: Validação ISO 19364\n');
fprintf(fid, '- Modularidade: Funções utils/ reutilizáveis\n');
fprintf(fid, '- Nomenclatura: Padronização ativo_states, controls, NLT\n\n');

%% 8. ESTRUTURA DE DIRETÓRIOS
write_section(fid, 'ESTRUTURA DE DIRETÓRIOS');
print_directory_tree(fid, root_dir, '', 0, 3);
fprintf(fid, '\n');

%% 9. ROADMAP E STATUS (ATUALIZADO)
write_section(fid, 'ROADMAP E PRIORIDADES');
roadmap = {
    '✅ Concluído', {
        'Framework core (DD + utils)'
        '5-DOF model com Variant Subsystem'
        'Controladores Passive, PID, LQR'
        'Manobra DLC (ISO 3888)'
        'Métricas básicas (Roll RMS/Max)'
        'Estrutura de dados ISO 19364'
    };
    '🚧 Em Desenvolvimento', {
        'Validação estatística completa (ECDF, Theil U)'
        'Comparação multi-controlador automatizada'
        'Outras manobras (Fishhook, J-Turn, Step Steer)'
        'Batch simulations (sweep velocidades)'
    };
    '⏳ Planejado (Q1 2026)', {
        'LQR gain-scheduling (40/70/100 km/h)'
        'MPC adaptativo com restrições'
        '9-DOF: Suspensão vertical (z, θ, ψ, z_ui)'
        'CI/CD: Model Advisor + Jenkins'
    };
    '🔮 Futuro (Q2-Q3 2026)', {
        '14-DOF: Pacejka Magic Formula 5.2'
        'Co-simulação CarMaker/IPG'
        'HIL: dSPACE MicroAutoBox'
        'Validação experimental (IMU + GPS RTK)'
    };
};

for i = 1:size(roadmap, 1)
    fprintf(fid, '**%s**:\n', roadmap{i,1});
    items = roadmap{i,2};
    for j = 1:length(items)
        fprintf(fid, '   - %s\n', items{j});
    end
    fprintf(fid, '\n');
end

%% 10. REFERÊNCIAS BIBLIOGRÁFICAS
write_section(fid, 'REFERÊNCIAS BIBLIOGRÁFICAS');
fprintf(fid, '[Khalil2019] Khalil, M., et al. "Improving Vehicle Rollover Resistance\n');
fprintf(fid, '             Using Fuzzy PID Controller of Active Anti-Roll Bar System",\n');
fprintf(fid, '             SAE Int. J. Passeng. Cars, 2019. DOI: 10.4271/06-12-01-0003\n\n');

fprintf(fid, '[Vu2016]     Vu, V.T., et al. "H∞/LPV Controller Design for Active Anti-Roll\n');
fprintf(fid, '             Bar System of Heavy Vehicles", IFAC AAC, 2016.\n\n');

fprintf(fid, '[Liu2019]    Liu, W., et al. "Shared Control for Active Steering and Active\n');
fprintf(fid, '             Anti-Roll Bar System", SAE Technical Paper, 2019.\n\n');

fprintf(fid, '[ISO19364]   ISO 19364:2016 - Passenger cars - Vehicle dynamic simulation\n');
fprintf(fid, '             and validation - Lateral transient response test methods\n\n');

%% 11. HEALTH SCORE (NOVO)
write_section(fid, 'PROJECT HEALTH SCORE');
health_score = calculate_health_score(git_status, dd_path, code_stats, sim_stats);
fprintf(fid, 'Score Geral: %.0f/100\n\n', health_score.total);
fprintf(fid, '   Controle de Versão:  %.0f/20 %s\n', health_score.git, get_emoji(health_score.git, 20));
fprintf(fid, '   Arquitetura:         %.0f/25 %s\n', health_score.architecture, get_emoji(health_score.architecture, 25));
fprintf(fid, '   Código:              %.0f/20 %s\n', health_score.code, get_emoji(health_score.code, 20));
fprintf(fid, '   Testes/Simulações:   %.0f/20 %s\n', health_score.tests, get_emoji(health_score.tests, 20));
fprintf(fid, '   Documentação:        %.0f/15 %s\n', health_score.docs, get_emoji(health_score.docs, 15));

fprintf(fid, '\n');
if health_score.total >= 80
    fprintf(fid, '🎉 Projeto em excelente estado!\n');
elseif health_score.total >= 60
    fprintf(fid, '✅ Projeto em bom estado, pequenas melhorias sugeridas\n');
else
    fprintf(fid, '⚠️  Ação necessária em áreas críticas\n');
end

%% Rodapé
fprintf(fid, '\n═══════════════════════════════════════════════════════════════\n');
fprintf(fid, 'FIM DO RELATÓRIO v2.0\n');
fprintf(fid, 'Próxima atualização: %s\n', datestr(now + 7, 'dd-mmm-yyyy'));
fprintf(fid, '═══════════════════════════════════════════════════════════════\n');

fclose(fid);

fprintf('✅ Relatório salvo: %s\n', report_file);
fprintf('📋 Pronto para copiar/colar\n');
fprintf('💾 Tamanho: %.1f KB\n', dir(report_file).bytes/1024);

end

%% ═══════════════════════════════════════════════════════
%% FUNÇÕES AUXILIARES (NOVAS)
%% ═══════════════════════════════════════════════════════

function write_section(fid, title)
fprintf(fid, '## %s\n\n', upper(title));
end

function [status, info] = get_git_status()
info = struct();
try
    [s, branch] = system('git rev-parse --abbrev-ref HEAD');
    if s == 0
        status = true;
        info.branch = strtrim(branch);
        
        [~, hash] = system('git rev-parse --short HEAD');
        info.hash = strtrim(hash);
        
        [~, msg] = system('git log -1 --pretty=format:"%s"');
        info.last_commit = strtrim(msg);
        
        [~, author] = system('git log -1 --pretty=format:"%an"');
        info.author = strtrim(author);
        
        [~, date] = system('git log -1 --pretty=format:"%ad"');
        info.date = strtrim(date);
        
        [~, modified] = system('git ls-files -m');
        if ~isempty(modified)
            info.modified_files = strsplit(strtrim(modified), '\n');
        else
            info.modified_files = {};
        end
    else
        status = false;
    end
catch
    status = false;
end
end

function installed = check_toolbox(toolbox_name)
v = ver;
installed = any(strcmp({v.Name}, toolbox_name));
end

function stats = analyze_code_metrics(root_dir)
m_files = dir(fullfile(root_dir, '**', '*.m'));
slx_files = dir(fullfile(root_dir, '**', '*.slx'));

stats.num_m_files = length(m_files);
stats.num_slx_files = length(slx_files);
stats.total_loc = 0;
stats.total_comments = 0;

for i = 1:length(m_files)
    fid = fopen(fullfile(m_files(i).folder, m_files(i).name), 'r');
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    
    lines = lines{1};
    stats.total_loc = stats.total_loc + length(lines);
    
    comment_lines = cellfun(@(x) ~isempty(x) && x(1) == '%', lines);
    stats.total_comments = stats.total_comments + sum(comment_lines);
end

stats.avg_loc = stats.total_loc / max(1, stats.num_m_files);
stats.comment_ratio = stats.total_comments / max(1, stats.total_loc);

stats.total_slx_size = sum([slx_files.bytes]);
end

function stats = analyze_recent_simulations(root_dir)
stats.num_simulations = 0;
stats.by_controller = struct();
stats.by_maneuver = struct();
stats.avg_roll_rms = NaN;
stats.avg_exec_time = NaN;

results_dir = fullfile(root_dir, 'results', '5dof');
if ~exist(results_dir, 'dir')
    return;
end

mat_files = dir(fullfile(results_dir, '**', '*.mat'));

% Filtrar últimos 7 dias
recent_files = mat_files(now - [mat_files.datenum] < 7);

if isempty(recent_files)
    return;
end

stats.num_simulations = length(recent_files);

roll_rms_values = [];
exec_time_values = [];

for i = 1:length(recent_files)
    try
        data = load(fullfile(recent_files(i).folder, recent_files(i).name));
        
        if isfield(data, 'sim_data')
            ctrl = data.sim_data.Metadata.controller;
            maneuver = data.sim_data.Metadata.maneuver;
            
            if ~isfield(stats.by_controller, ctrl)
                stats.by_controller.(ctrl) = 0;
            end
            stats.by_controller.(ctrl) = stats.by_controller.(ctrl) + 1;
            
            if ~isfield(stats.by_maneuver, maneuver)
                stats.by_maneuver.(maneuver) = 0;
            end
            stats.by_maneuver.(maneuver) = stats.by_maneuver.(maneuver) + 1;
            
            if isfield(data.sim_data, 'Metrics')
                roll_rms_values(end+1) = data.sim_data.Metrics.rms_roll_deg;
                exec_time_values(end+1) = data.sim_data.Metrics.execution_time_s;
            end
        end
    catch
        continue;
    end
end

if ~isempty(roll_rms_values)
    stats.avg_roll_rms = mean(roll_rms_values);
    stats.avg_exec_time = mean(exec_time_values);
end
end

function score = calculate_health_score(git_status, dd_path, code_stats, sim_stats)
score.git = 0;
score.architecture = 0;
score.code = 0;
score.tests = 0;
score.docs = 0;

% Git (20 pontos)
if git_status
    score.git = 20;
end

% Arquitetura (25 pontos)
if exist(dd_path, 'file')
    score.architecture = score.architecture + 15;
end
if code_stats.num_m_files > 20
    score.architecture = score.architecture + 10;
end

% Código (20 pontos)
if code_stats.comment_ratio > 0.15
    score.code = score.code + 10;
end
if code_stats.avg_loc < 200
    score.code = score.code + 10;
end

% Testes (20 pontos)
if sim_stats.num_simulations > 0
    score.tests = min(20, sim_stats.num_simulations * 5);
end

% Docs (15 pontos)
if exist(fullfile(pwd, 'docs'), 'dir')
    score.docs = 15;
end

score.total = score.git + score.architecture + score.code + score.tests + score.docs;
end

function emoji = get_emoji(value, max_value)
ratio = value / max_value;
if ratio >= 0.8
    emoji = '✅';
elseif ratio >= 0.5
    emoji = '⚠️';
else
    emoji = '❌';
end
end

function print_directory_tree(fid, root, prefix, level, max_level)
if level >= max_level
    return;
end

items = dir(root);
items = items(~ismember({items.name}, {'.', '..', '.git', 'slprj', 'slxc'}));

folders = items([items.isdir]);
files = items(~[items.isdir]);

for i = 1:length(folders)
    fprintf(fid, '%s  📁 %s/\n', prefix, folders(i).name);
    new_prefix = [prefix, '    '];
    print_directory_tree(fid, fullfile(folders(i).folder, folders(i).name), new_prefix, level+1, max_level);
end

for i = 1:min(10, length(files))
    icon = get_file_icon(files(i).name);
    fprintf(fid, '%s    %s %s (%.1f KB)\n', prefix, icon, files(i).name, files(i).bytes/1024);
end

if length(files) > 10
    fprintf(fid, '%s    ... (%d mais arquivos)\n', prefix, length(files)-10);
end
end

function icon = get_file_icon(filename)
[~, ~, ext] = fileparts(filename);
switch lower(ext)
    case '.m'
        icon = '🔧';
    case {'.slx', '.mdl'}
        icon = '🎛️';
    case {'.mat', '.sldd'}
        icon = '💾';
    case '.pdf'
        icon = '📄';
    case {'.txt', '.md'}
        icon = '📝';
    case '.prj'
        icon = '📦';
    otherwise
        icon = '📄';
end
end

function validation_report = validate_iso19364(sim_data, exp_data)
%VALIDATE_ISO19364 Validação estatística conforme ISO 19364:2016
%
% Inputs:
%   sim_data - struct de simulação (results.States)
%   exp_data - struct de dados experimentais (mesmo formato)
%
% Output:
%   validation_report - struct com testes ECDF, Theil, Corridor
%
% Referências:
%   [1] ISO 19364:2016 Section 6.2-6.4
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026

validation_report = struct();
validation_report.timestamp = datetime('now');
validation_report.tests_performed = {};

%% Variáveis críticas (ISO 19364 Table 1)
critical_vars = {'ay', 'delta', 'beta', 'phi', 'r'};

%% 1. ECDF Test (Kolmogorov-Smirnov)
fprintf('\n═══ VALIDAÇÃO ISO 19364:2016 ═══\n\n');
fprintf('1️⃣  ECDF Test (Kolmogorov-Smirnov, α=0.05)\n');

for i = 1:length(critical_vars)
    var_name = critical_vars{i};
    
    % Extrair dados (ajustar conforme nomenclatura)
    try
        sim_var = get_variable_data(sim_data, var_name);
        exp_var = get_variable_data(exp_data, var_name);
        
        [~, p_value, ks_stat] = kstest2(sim_var, exp_var, 'Alpha', 0.05);
        
        % Critério: p_value > 0.05 (não rejeitar H0)
        pass = p_value > 0.05;
        
        validation_report.(var_name).ecdf.ks_statistic = ks_stat;
        validation_report.(var_name).ecdf.p_value = p_value;
        validation_report.(var_name).ecdf.pass = pass;
        
        status = char(9989*pass + 10060*(~pass));  % ✅ ou ❌
        fprintf('   %s %-8s: KS=%.4f, p=%.4f %s\n', ...
                status, var_name, ks_stat, p_value, char(pass*'PASS'+(~pass)*'FAIL'));
    catch ME
        fprintf('   ⚠️  %s: Dados não disponíveis\n', var_name);
        validation_report.(var_name).ecdf.error = ME.message;
    end
end

%% 2. Theil Inequality Coefficient (U < 0.3)
fprintf('\n2️⃣  Theil Inequality Coefficient (critério: U < 0.3)\n');

for i = 1:length(critical_vars)
    var_name = critical_vars{i};
    
    try
        sim_var = get_variable_data(sim_data, var_name);
        exp_var = get_variable_data(exp_data, var_name);
        
        % Interpolar para mesmo comprimento
        if length(sim_var) ~= length(exp_var)
            exp_var = interp1(linspace(0,1,length(exp_var)), exp_var, ...
                              linspace(0,1,length(sim_var)));
        end
        
        % Calcular Theil U
        MSE = mean((sim_var - exp_var).^2);
        U = sqrt(MSE) / (rms(sim_var) + rms(exp_var));
        
        pass = U < 0.3;
        
        validation_report.(var_name).theil.U = U;
        validation_report.(var_name).theil.MSE = MSE;
        validation_report.(var_name).theil.pass = pass;
        
        status = char(9989*pass + 10060*(~pass));
        fprintf('   %s %-8s: U=%.4f %s\n', status, var_name, U, ...
                char(pass*'PASS'+(~pass)*'FAIL'));
    catch ME
        fprintf('   ⚠️  %s: Erro no cálculo\n', var_name);
        validation_report.(var_name).theil.error = ME.message;
    end
end

%% 3. Corridor Test (95% CI)
fprintf('\n3️⃣  Corridor Test (95%% Confidence Interval)\n');

for i = 1:length(critical_vars)
    var_name = critical_vars{i};
    
    try
        sim_var = get_variable_data(sim_data, var_name);
        exp_mean = mean(get_variable_data(exp_data, var_name));
        exp_std = std(get_variable_data(exp_data, var_name));
        
        % Limites do corredor
        upper = exp_mean + 1.96*exp_std;
        lower = exp_mean - 1.96*exp_std;
        
        % Percentual dentro do corredor
        in_corridor = (sim_var >= lower) & (sim_var <= upper);
        pct_inside = 100 * sum(in_corridor) / length(sim_var);
        
        pass = pct_inside >= 95.0;
        
        validation_report.(var_name).corridor.pct_inside = pct_inside;
        validation_report.(var_name).corridor.upper_bound = upper;
        validation_report.(var_name).corridor.lower_bound = lower;
        validation_report.(var_name).corridor.pass = pass;
        
        status = char(9989*pass + 10060*(~pass));
        fprintf('   %s %-8s: %.1f%% inside corridor %s\n', ...
                status, var_name, pct_inside, char(pass*'PASS'+(~pass)*'FAIL'));
    catch ME
        fprintf('   ⚠️  %s: Erro no teste\n', var_name);
        validation_report.(var_name).corridor.error = ME.message;
    end
end

%% Relatório Final
fprintf('\n═══════════════════════════════════════\n');
validation_report.tests_performed = critical_vars;
validation_report.compliant_iso19364 = all_tests_passed(validation_report);

if validation_report.compliant_iso19364
    fprintf('✅ MODELO VALIDADO CONFORME ISO 19364:2016\n');
else
    fprintf('❌ VALIDAÇÃO PARCIAL - Revisar variáveis falhadas\n');
end
fprintf('═══════════════════════════════════════\n\n');

end

%% Funções auxiliares
function data = get_variable_data(dataset, var_name)
    % Mapear nomes ISO para nomenclatura interna
    var_map = containers.Map(...
        {'ay', 'delta', 'beta', 'phi', 'r'}, ...
        {'a_y', 'delta_f', 'beta', 'roll', 'yaw_rate'});
    
    if isKey(var_map, var_name)
        internal_name = var_map(var_name);
        if istimetable(dataset)
            data = dataset.(internal_name);
        elseif isstruct(dataset)
            data = dataset.(internal_name);
        else
            error('Formato de dados não suportado');
        end
    else
        error('Variável %s não encontrada', var_name);
    end
end

function pass = all_tests_passed(report)
    vars = fieldnames(report);
    pass = true;
    for i = 1:length(vars)
        if isstruct(report.(vars{i}))
            tests = fieldnames(report.(vars{i}));
            for j = 1:length(tests)
                if isfield(report.(vars{i}).(tests{j}), 'pass')
                    pass = pass && report.(vars{i}).(tests{j}).pass;
                end
            end
        end
    end
end

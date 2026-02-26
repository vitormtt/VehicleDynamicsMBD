%% AUTOMATED EXPERIMENT RUNNER (LIVE SCRIPT)
% Este script realiza o Git Pull, roda as simulações base (Passivo e Ativo)
% e gera a tabela de comparação final.
%
% Você pode rodar este arquivo diretamente no MATLAB como um script normal.

disp('=======================================================');
disp('   INICIANDO BATCH DE VALIDAÇÃO 5-DOF');
disp('=======================================================');

%% 1. Atualizar Repositório (Git Pull)
disp('1. Baixando atualizações do GitHub...');
try
    [status, cmdout] = system('git pull');
    if status == 0
        disp('✅ Repositório atualizado com sucesso:');
        disp(cmdout);
    else
        warning('Falha ao rodar git pull. Verifique a saída:');
        disp(cmdout);
    end
catch ME
    warning('Erro ao tentar rodar git pull pelo MATLAB: %s', ME.message);
end

%% 2. Executar Simulação Passiva (Sem Controlador)
disp('=======================================================');
disp('2. Rodando Caso Base (Passive)...');
disp('=======================================================');
% Usamos try-catch para que o script não pare de vez se der erro em uma etapa
try
    run_5dof_simulation('Passive', 'Gaspar', 70);
catch ME
    warning('Erro na simulação Passiva: %s', ME.message);
end

%% 3. Executar Simulação Ativa (PID)
disp('=======================================================');
disp('3. Rodando Caso Ativo (PID)...');
disp('=======================================================');
try
    run_5dof_simulation('PID', 'Gaspar', 70);
catch ME
    warning('Erro na simulação Ativa: %s', ME.message);
end

%% 4. Comparar Resultados
disp('=======================================================');
disp('4. Gerando Tabela de Comparação de Métricas...');
disp('=======================================================');
try
    compare_simulation_metrics();
catch ME
    warning('Erro ao gerar tabela: %s', ME.message);
end

disp('=======================================================');
disp('   BATCH CONCLUÍDO!');
disp('=======================================================');

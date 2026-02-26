function setup_model_logging(model_name)
%SETUP_MODEL_LOGGING Configura logging padronizado ISO 19364
%
% Usage:
%   setup_model_logging('model_5dof')
%   setup_model_logging('model_9dof')
%
% Autor: Vitor Yukio - UnB/PIBIC
% Data: 13/02/2026

    % Carregar modelo se necessário
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end
    
    % Configuração básica
    set_param(model_name, 'SignalLogging', 'on');
    set_param(model_name, 'SignalLoggingName', 'logsout');
    set_param(model_name, 'SaveOutput', 'on');
    set_param(model_name, 'OutputSaveName', 'yout');
    set_param(model_name, 'SaveState', 'on');
    set_param(model_name, 'StateSaveName', 'xout');
    set_param(model_name, 'SignalLoggingSaveFormat', 'Dataset');
    
    % Compilar para ativar variantes
    set_param(model_name, 'SimulationCommand', 'update');
    
    % Configurar State-Space
    ss_blocks = find_system(model_name, ...
        'MatchFilter', @Simulink.match.allVariants, ...
        'BlockType', 'StateSpace');
    
    for i = 1:length(ss_blocks)
        try
            ph = get_param(ss_blocks{i}, 'PortHandles');
            if ~isempty(ph.Outport)
                set_param(ph.Outport(1), 'DataLogging', 'on');
                set_param(ph.Outport(1), 'DataLoggingName', 'ativo_states');
            end
        catch
            % Ignorar blocos sem porta
        end
    end
    
    save_system(model_name);
    fprintf('✅ Logging configurado em %s\n', model_name);
end

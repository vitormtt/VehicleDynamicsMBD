% utils/configure_signal_logging.m
function configure_signal_logging(modelName)
    load_system(modelName);
    
    % Config Parameters → Data Import/Export
    set_param(modelName, 'SignalLogging', 'on');
    set_param(modelName, 'SignalLoggingName', 'logsout');
    set_param(modelName, 'SaveFormat', 'Dataset');
    set_param(modelName, 'SaveOutput', 'on');
    set_param(modelName, 'OutputSaveName', 'yout');
    set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
    
    % Marca sinais específicos para logging
    stateSpaceBlock = [modelName '/StateSpace'];
    
    % Cria signal logging para output do StateSpace
    portHandles = get_param(stateSpaceBlock, 'PortHandles');
    outPort = portHandles.Outport(1);
    
    % Configura logging (R2024b)
    set(outPort, 'DataLogging', 1);
    set(outPort, 'DataLoggingName', 'vehicle_states');
    set(outPort, 'DataLoggingNameMode', 'Custom');
    
    save_system(modelName);
    fprintf('✅ Signal logging configurado: %s\n', modelName);
end

function states = extract_states_from_dataset(xout)
%EXTRACT_STATES_FROM_DATASET Extrair estados de Simulink Dataset
%
% Input:  xout - Simulink.SimulationData.Dataset
% Output: states - matriz [T×6] com estados do modelo 5-DOF
%
% Autor: Vitor Yukio - UnB/PIBIC

if ~isa(xout, 'Simulink.SimulationData.Dataset')
    error('xout deve ser Simulink.SimulationData.Dataset');
end

states = [];

%% Procurar elemento com 6 colunas (State-Space)
for i = 1:xout.numElements
    elem = xout{i};
    data = elem.Values.Data;
    
    % State-Space tem 6 colunas (5-DOF model)
    if size(data, 2) == 6
        states = data;
        return;
    end
end

%% Fallback: buscar pelo nome do bloco
if isempty(states)
    for i = 1:xout.numElements
        elem = xout{i};
        blockPath = elem.BlockPath.getBlock(1);
        if contains(blockPath, 'Espaço de Estados')
            states = elem.Values.Data;
            return;
        end
    end
end

if isempty(states)
    error('Estado do State-Space não encontrado (esperado 6 colunas)');
end

end

function build_truck_model()
    model = 'Truck_5DOF_ARB';
    
    % Limpa modelo
    blocks = find_system(model, 'SearchDepth', 1, 'Type', 'Block');
    for i = 2:length(blocks)  % Pula modelo root
        delete_block(blocks{i});
    end
    
    % 1. From Workspace (steering input)
    add_block('simulink/Sources/From Workspace', [model '/steering_signal']);
    set_param([model '/steering_signal'], ...
        'VariableName', 'steering_signal', ...
        'Position', [50 100 150 130]);
    
    % 2. Constant (feedback inicial = 0)
    add_block('simulink/Sources/Constant', [model '/beta_init']);
    set_param([model '/beta_init'], 'Value', '0', 'Position', [50 200 100 230]);
    
    add_block('simulink/Sources/Constant', [model '/r_init']);
    set_param([model '/r_init'], 'Value', '0', 'Position', [50 260 100 290]);
    
    % 3. Mux (3 inputs para tire_forces)
    add_block('simulink/Signal Routing/Mux', [model '/Mux_tire_in']);
    set_param([model '/Mux_tire_in'], 'Inputs', '3', 'Position', [180 140 185 280]);
    
    % 4. MATLAB Function (tire_forces)
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model '/tire_forces']);
    set_param([model '/tire_forces'], 'Position', [240 190 340 240]);
    
    % 5. State-Space (lê A/B/C/D do DD)
    add_block('simulink/Continuous/State-Space', [model '/StateSpace']);
    set_param([model '/StateSpace'], ...
        'A', 'A_passive', 'B', 'B_passive', ...
        'C', 'C_passive', 'D', 'D_passive', ...
        'Position', [420 180 520 250]);
    
    % 6. To Workspace (salva estados)
    add_block('simulink/Sinks/To Workspace', [model '/out_states']);
    set_param([model '/out_states'], ...
        'VariableName', 'states_out', ...
        'SaveFormat', 'Timeseries', ...
        'Position', [600 200 700 230]);
    
    % 7. Conexões
    add_line(model, 'steering_signal/1', 'Mux_tire_in/1');
    add_line(model, 'beta_init/1', 'Mux_tire_in/2');
    add_line(model, 'r_init/1', 'Mux_tire_in/3');
    add_line(model, 'Mux_tire_in/1', 'tire_forces/1');
    add_line(model, 'tire_forces/1', 'StateSpace/1');
    add_line(model, 'StateSpace/1', 'out_states/1');
    
    save_system(model);
    fprintf('✅ Modelo construído (6 blocos + conexões)\n');
    fprintf('⚠️  Configure tire_forces code manualmente!\n');
end

function sim_data = create_simulation_structure(simOut, config, metrics)
%CREATE_SIMULATION_STRUCTURE Estrutura padronizada ISO 19364
%
% Autor: Vitor Yukio - UnB/PIBIC

sim_data = struct();

%% METADATA
sim_data.Metadata = struct();
sim_data.Metadata.timestamp = datetime('now', 'TimeZone', 'America/Sao_Paulo');
sim_data.Metadata.model_version = '2.0';
sim_data.Metadata.vehicle = config.vehicle_name;
sim_data.Metadata.controller = config.controller;
sim_data.Metadata.maneuver = config.maneuver;
sim_data.Metadata.velocity_kmh = config.velocity;
sim_data.Metadata.solver = config.solver;
sim_data.Metadata.RelTol = config.reltol;

try
    [status, git_hash] = system('git rev-parse --short HEAD');
    if status == 0
        sim_data.Metadata.git_commit = strtrim(git_hash);
    else
        sim_data.Metadata.git_commit = 'unknown';
    end
catch
    sim_data.Metadata.git_commit = 'unknown';
end

%% INPUTS
if isfield(config, 'steering_signal') && ~isempty(config.steering_signal)
    sim_data.Inputs = create_inputs_timetable(simOut.tout, config.steering_signal);
else
    sim_data.Inputs = [];
end

%% STATES
states = [];

if isprop(simOut, 'xout') && ~isempty(simOut.xout)
    if isa(simOut.xout, 'Simulink.SimulationData.Dataset')
        states = extract_states_from_dataset(simOut.xout);
        fprintf('   Estados extraídos: xout (Dataset) [%s]\n', mat2str(size(states)));
    else
        states = simOut.xout;
        fprintf('   Estados extraídos: xout (matriz) [%s]\n', mat2str(size(states)));
    end
elseif isprop(simOut, 'yout') && ~isempty(simOut.yout)
    if isa(simOut.yout, 'timeseries')
        states = simOut.yout.Data;
    else
        states = simOut.yout;
    end
    fprintf('   Estados extraídos: yout [%s]\n', mat2str(size(states)));
end

if ~isempty(states)
    if size(states, 1) == 6
        states = states';
    end
    sim_data.States = create_states_timetable(simOut.tout, states);
else
    warning('Estados não encontrados');
    sim_data.States = [];
end

%% CONTROLS
n_samples = length(simOut.tout);
sim_data.Controls = create_controls_timetable(simOut.tout, zeros(n_samples, 2));

%% METRICS
sim_data.Metrics = metrics;

%% SIMULATION INFO
sim_data.SimulationInfo = struct();
if isprop(simOut, 'ExecutionInfo')
    sim_data.SimulationInfo.elapsed_time_s = simOut.ExecutionInfo.ExecutionTime;
else
    sim_data.SimulationInfo.elapsed_time_s = NaN;
end
sim_data.SimulationInfo.num_steps = length(simOut.tout);

end

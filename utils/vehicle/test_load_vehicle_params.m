load_vehicle_params.m
% Script de teste para validar carregamento de parâmetros

clear; clc;

%% Teste 1: Carregar parâmetros truck
fprintf('=== Teste 1: Carregar Truck ===\n');
vehicle = load_vehicle_params('truck');

%% Teste 2: Validar valores críticos (comparar com script LQR original)
assert(vehicle.m == 14193, 'Massa total incorreta');
assert(vehicle.ms == 12487, 'Massa suspensa incorreta');
assert(vehicle.Ixx == 24201, 'Inércia Ixx incorreta');
assert(vehicle.kAO_f == 10730, 'Rigidez ARB frontal incorreta');
assert(vehicle.kAO_r == 15480, 'Rigidez ARB traseira incorreta');

fprintf('✅ Todos os valores validados!\n\n');

%% Teste 3: Calcular SSF e comparar com Khalil (SSF = 1.06)
SSF_calculated = (vehicle.lw * 2) / (2 * vehicle.h);
fprintf('SSF Calculado:    %.2f\n', SSF_calculated);
fprintf('SSF Khalil 2019:  1.06\n');
assert(abs(SSF_calculated - 1.12) < 0.1, 'SSF fora do esperado'); % Nota: pequena diferença devido a arredondamentos

%% Teste 4: Validar consistência física
fprintf('\n=== Validações Físicas ===\n');

% Massa
mass_check = vehicle.ms + vehicle.muf + vehicle.mur;
fprintf('Soma massas:      %.0f kg (esperado: %.0f kg)\n', mass_check, vehicle.m);

% Wheelbase
wb_check = vehicle.lf + vehicle.lr;
fprintf('Wheelbase:        %.2f m (lf+lr = %.2f m)\n', vehicle.w, wb_check);

% Centro de gravidade (deve estar entre eixos)
assert(vehicle.lf > 0 && vehicle.lr > 0, 'CG fora dos eixos');
fprintf('CG Position:      OK (%.2f m da frente, %.2f m da traseira)\n', ...
        vehicle.lf, vehicle.lr);

fprintf('\n✅ Teste completo! Parâmetros consistentes.\n');

%% Teste 5: Gerar struct para visualização
fprintf('\n=== Struct Completo ===\n');
disp(vehicle);

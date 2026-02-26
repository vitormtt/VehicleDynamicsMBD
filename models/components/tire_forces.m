% Function: tire_forces.m
% Description: Calculate tire lateral forces using slip angles
function Fy  = tire_forces(u)
    % Descrição das Entradas
    beta = u(3);        % Ângulo de Deriva
    psi_dot = u(2);     % Taxa de Guinada
    delta_f = u(1);     % Ângulo de Esterçamento
    
    % Parâmetros do veículo (Caminhão Pesado)
    lf = 1.95;           % Distância do CG ao eixo dianteiro (m)
    lr = 1.54;           % Distância do CG ao eixo traseiro (m)
    Cf = 582000;         % Rigidez lateral do pneu dianteiro (N/rad)
    Cr = 783000;         % Rigidez lateral do pneu traseiro (N/rad)
    mu = 1;              % Coeficiente de aderência ao solo
    v = 70 / 3.6;        % Velocidade longitudinal (m/s)
    
    % Calcular ângulos de deriva
    alpha_f = -beta + delta_f - ((lf * psi_dot) / v);
    alpha_r = -beta + ((lr * psi_dot) / v);
    
    % Calcular forças laterais nos pneus (saídas escalares)
    Fyf = mu * Cf * alpha_f;
    Fyr = mu * Cr * alpha_r;
    
    % Combinar as saídas como vetor coluna (2x1)
    Fy = [Fyf; Fyr];
end


function Fy = pacejkaLateralForce(alpha, params)
% pacejkaLateralForce Calcula a força lateral do pneu usando a Fórmula Mágica
% de Pacejka, com base na Eq. (8) do artigo de Hassan et al. (2021).
%
% Entradas:
%   alpha  - Ângulo de escorregamento (sideslip angle) em radianos. Pode ser um vetor.
%   params - Estrutura (struct) contendo os coeficientes do modelo Pacejka:
%            params.By  (Fator de Rigidez Lateral, B)
%            params.Cy  (Fator de Forma Lateral, C)
%            params.Dy  (Fator de Pico Lateral, D)
%            params.Ey  (Fator de Curvatura Lateral, E)
%            params.Shy (Deslocamento Horizontal, Sh)
%            params.Svy (Deslocamento Vertical, Sv)
%
% Saída:
%   Fy     - Força lateral do pneu (Fy) em Newtons.

% Extração dos parâmetros para maior clareza
B = params.By;
C = params.Cy;
D = params.Dy;
E = params.Ey;
Sh = params.Shy;
Sv = params.Svy;

% Argumento do arco-tangente (Eq. 8, parte 2)
phi = (1 - E) .* (alpha + Sh) + (E / B) .* atan(B .* (alpha + Sh));

% Fórmula Mágica (Eq. 8, parte 1)
% O fator de pico D (Dy) está em N, então a saída Fy está em N.
Fy = D .* sin(C .* atan(B .* phi)) + Sv;

end

%[appendix]{"version":"1.0"}
%---

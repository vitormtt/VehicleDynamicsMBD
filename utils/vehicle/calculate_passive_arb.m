function [MAR] = calculate_passive_arb(kAO, tA, tB, c)
%CALCULATE_PASSIVE_ARB Calcula momentos da barra estabilizadora passiva
%
% Sintaxe:
%   MAR = calculate_passive_arb(kAO, tA, tB, c)
%
% Entradas:
%   kAO - Rigidez torcional da ARB (Nm/rad)
%   tA  - Braço principal (m)
%   tB  - Braço secundário (m)
%   c   - Fator geométrico (altura) (m)
%
% Saídas:
%   MAR - Struct com campos:
%         .phi    - Momento aplicado à massa suspensa
%         .phi_u  - Momento aplicado à massa não suspensa
%
% Referência:
%   Khalil et al. (2019), SAE 2019-01-0003, Eqs. 6-9
%
% Exemplo:
%   MAR_f = calculate_passive_arb(2500, 0.766, 0.550, 0.285);
%
% Vitor Yukio - UnB/PIBIC - 11/02/2026

    % Validação de entradas
    validateattributes(kAO, {'numeric'}, {'positive', 'scalar'});
    validateattributes(tA, {'numeric'}, {'positive', 'scalar'});
    validateattributes(tB, {'numeric'}, {'positive', 'scalar'});
    validateattributes(c, {'numeric'}, {'positive', 'scalar'});
    
    % Cálculo (Eq. 31 Khalil)
    MAR.phi = 4 * kAO * (tA * tB) / c^2;
    MAR.phi_u = -4 * kAO * (tA^2) / c^2;
    
    % Log para debug (opcional)
    if nargout == 0
        fprintf('ARB Stiffness: kAO = %.1f Nm/rad\n', kAO);
        fprintf('Sprung mass moment:   %.1f Nm/rad\n', MAR.phi);
        fprintf('Unsprung mass moment: %.1f Nm/rad\n', MAR.phi_u);
    end
end

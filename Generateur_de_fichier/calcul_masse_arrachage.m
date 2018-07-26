function masse_max_poussee = calcul_masse_arrachage(coefficient_poussee,abscisse_max)
%CALCUL_MASSE_ARRACHAGE
%   Calcule de la masse maximale que génére les moteurs à partir du
%   coefficient de poussée et de la rotation au carrée maximale
    g = 9.80665; % m/s²
    masse_max_poussee = 4*(coefficient_poussee * abscisse_max)/g;
end


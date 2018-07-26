function masse_max_poussee = calcul_masse_arrachage(coefficient_poussee,abscisse_max)
%CALCUL_MASSE_ARRACHAGE
%   Calcule de la masse maximale que g�n�re les moteurs � partir du
%   coefficient de pouss�e et de la rotation au carr�e maximale
    g = 9.80665; % m/s�
    masse_max_poussee = 4*(coefficient_poussee * abscisse_max)/g;
end


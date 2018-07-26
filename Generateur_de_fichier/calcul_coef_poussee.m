function coefficient_poussee = calcul_coef_poussee(rot_carree,force_par_moteur)
%CALCUL_COEF_POUSSEE
%   Fonction qui calcule le coefficient de pouss�e � partir des moindres
%   au carr�e, de la rotation des moteurs au carr�e et de la pouss�e g�n�r�
%   par un moteur
coefficient_poussee = rot_carree \ force_par_moteur;
end


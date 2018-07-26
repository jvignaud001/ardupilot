function coefficient_poussee = calcul_coef_poussee(rot_carree,force_par_moteur)
%CALCUL_COEF_POUSSEE
%   Fonction qui calcule le coefficient de poussée à partir des moindres
%   au carrée, de la rotation des moteurs au carrée et de la poussée généré
%   par un moteur
coefficient_poussee = rot_carree \ force_par_moteur;
end


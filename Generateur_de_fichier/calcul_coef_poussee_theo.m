function coefficient_poussee = calcul_coef_poussee_theo(pas_helice,diametre_helice)
%CALCUL_COEF_POUSSEE
%   Fonction qui calcule le coefficient de poussée à partir des moindres
%   au carrée, de la rotation des moteurs au carrée et de la poussée généré
%   par un moteur
coefficient_poussee = 28.35 * pas_helice * diametre_helice^3 * 10^(-10);
end


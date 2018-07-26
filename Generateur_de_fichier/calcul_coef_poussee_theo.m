function coefficient_poussee = calcul_coef_poussee_theo(pas_helice,diametre_helice)
%CALCUL_COEF_POUSSEE
%   Fonction qui calcule le coefficient de pouss�e � partir des moindres
%   au carr�e, de la rotation des moteurs au carr�e et de la pouss�e g�n�r�
%   par un moteur
coefficient_poussee = 28.35 * pas_helice * diametre_helice^3 * 10^(-10);
end


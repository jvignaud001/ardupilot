function coefficient_trainee = calcul_coef_trainee(temps_releve, rotation_maximale, moment_inertie_z)
%CALCUL_COEF_TRAINEE 
%   Fonction qui calcule le coefficient de trainée via les temps relevés
%   expérimentalement, ainsi que la rotation maximale et le moment
%   d'inertie en Z.
    somme_quart_periode = 0;
    for i=2:length(temps_releve)
        quart_periode = (temps_releve(i) - temps_releve(i-1))/4;
        somme_quart_periode = somme_quart_periode + quart_periode;
    end
    moyenne_quart_periode = somme_quart_periode/(length(temps_releve)-1);
    %% calcul de la rotation mi-puissance
    rotation_mi_puissance = rotation_maximale/2;
    %% coefficient de trainée
    coefficient_trainee = pi*moment_inertie_z/(2*rotation_mi_puissance^2*moyenne_quart_periode^2);
end


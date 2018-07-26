function [abscisse_poussee,ordonnee_poussee] = calcul_abscisse_ordonnee_poussee(releve_pourcent,releve_masse,rotation_maximale,nbMoteurs)
%CALCUL_ABSCISSE_ORDONNEE_POUSSEE 
%   On calcule l'abscisse (la rotation au carrée des moteurs) et l'ordonnée
%   (la force générée par un moteur) lors des tests de poussée.
    
    %% calcul abscisse : rotation au carrée
    rotation_max_carree = rotation_maximale^2;
    abscisse_poussee = releve_pourcent.'/100 .* rotation_max_carree ; % abscisse : rotation au carrée des moteurs

    %% calcul ordonnée : force de poussée
    g = 9.80665; % en m/s^2
    releve_masse = releve_masse(1) - releve_masse; % on obtient la différence de masse (masse de l'ensemble - masse relevé lors de la poussée des moteurs)
    force_poussee1 = releve_masse./1000 * g; % conversion en Newton
    force_poussee_moteurs1 = force_poussee1 / nbMoteurs; % force par moteurs
    ordonnee_poussee = force_poussee_moteurs1.'; % transposée
end


function [abscisse_poussee,ordonnee_poussee] = calcul_abscisse_ordonnee_poussee(releve_pourcent,releve_masse,rotation_maximale,nbMoteurs)
%CALCUL_ABSCISSE_ORDONNEE_POUSSEE 
%   On calcule l'abscisse (la rotation au carr�e des moteurs) et l'ordonn�e
%   (la force g�n�r�e par un moteur) lors des tests de pouss�e.
    
    %% calcul abscisse : rotation au carr�e
    rotation_max_carree = rotation_maximale^2;
    abscisse_poussee = releve_pourcent.'/100 .* rotation_max_carree ; % abscisse : rotation au carr�e des moteurs

    %% calcul ordonn�e : force de pouss�e
    g = 9.80665; % en m/s^2
    releve_masse = releve_masse(1) - releve_masse; % on obtient la diff�rence de masse (masse de l'ensemble - masse relev� lors de la pouss�e des moteurs)
    force_poussee1 = releve_masse./1000 * g; % conversion en Newton
    force_poussee_moteurs1 = force_poussee1 / nbMoteurs; % force par moteurs
    ordonnee_poussee = force_poussee_moteurs1.'; % transpos�e
end

